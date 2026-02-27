#include <iostream>
#include <cstring>
#include <unistd.h>
#include <vector>
#include <string>
#include <mutex>

#include <opencv2/opencv.hpp>

using namespace std;

#ifndef _WIN32
    #define __stdcall
    #define CALLINGCONVEN
    #define CNET_APIIMPORT
    #define CALLBACK
    #define WINAPI
    
    typedef unsigned long DWORD;
    typedef unsigned short WORD;
    typedef unsigned char BYTE;
    typedef long LPARAM;
    typedef unsigned long WPARAM;
    typedef int BOOL;
    
    typedef unsigned int UINT;
    typedef void* HWND;
    typedef void* HANDLE;
    typedef void* HDC;
    typedef unsigned int COLORREF;
    typedef long LONG;
    
    typedef struct _RECT {
        LONG left;
        LONG top;
        LONG right;
        LONG bottom;
    } RECT;

    #ifndef TRUE
        #define TRUE 1
    #endif
    #ifndef FALSE
        #define FALSE 0
    #endif
#endif

#include "LinuxDef.h"
#include "InfraredTempSDK.h"

// --- [전역 변수] ---
static bool g_tempSaved = false;
std::mutex g_frameMutex;
cv::Mat g_rawYuvFrame;   // 콜백에서 원본 데이터를 빠르게 저장할 공간
bool g_hasNewFrame = false;

// --- [영상 데이터 콜백 함수: 초고속 최적화] ---
void videoCallBack(char *pBuffer, long BufferLen, int width, int height, void *pContext) {
    // YUV420 포맷 확인 (384x288x1.5 = 165888)
    if (BufferLen == (width * height * 3 / 2)) {
        // 스레드 경합 최소화를 위해 lock은 데이터 복사 시에만 짧게 사용
        std::lock_guard<std::mutex> lock(g_frameMutex);
        
        // 1. 수신된 버퍼를 즉시 Mat 객체로 래핑
        cv::Mat yuvWrapper(height * 3 / 2, width, CV_8UC1, pBuffer);
        
        // 2. 무거운 변환 연산 없이 메모리만 통째로 복사 (매우 빠름)
        yuvWrapper.copyTo(g_rawYuvFrame);
        
        g_hasNewFrame = true;
    }
}

// 온도 데이터 콜백 함수
void tempCallBack(char *pBuffer, long BufferLen, void* pContext) {
    if (g_tempSaved) return; 
    g_tempSaved = true;
    std::cout << "[Temp] First temperature data received.\n";
}

int main() {
    std::cout << "Initializing SDK...\n";
    
    int deviceType = 1; 
    char username[] = "admin";
    char password[] = "admin";
    
    sdk_set_type(deviceType, username, password);
    if (sdk_initialize() < 0) {
        std::cerr << "SDK Initialization failed!\n";
        return -1;
    }

    sleep(1); 
    IRNETHANDLE pHandle = sdk_create();
    if (pHandle == NULL) {
        std::cerr << "SDK handle creation failed!\n";
        return -1;
    }

    ChannelInfo devInfo;
    memset(&devInfo, 0, sizeof(ChannelInfo));
    strcpy(devInfo.szUserName, username);
    strcpy(devInfo.szPWD, password);
    const char* targetIP = "192.168.1.123";
    strcpy(devInfo.szIP, targetIP);
    devInfo.wPortNum = 3000;

    if (sdk_loginDevice(pHandle, devInfo) != 0) {
        std::cerr << "Login failed!\n";
        sdk_release();
        return -1;
    }

    SetDeviceVideoCallBack(pHandle, videoCallBack, nullptr);
    SetTempCallBack(pHandle, tempCallBack, nullptr);

    sdk_start_url(pHandle, devInfo.szIP);

    std::cout << "\n=====================================\n";
    std::cout << " [실시간 최적화 모드 작동 중] \n";
    std::cout << " 1 : 흑백 모드 / 2 : 컬러 모드\n";
    std::cout << " ESC : 종료\n";
    std::cout << "=====================================\n";

    int displayMode = 1; // 1: Grayscale, 2: Inferno Color

    while (true) {
        cv::Mat rawYuv;

        // 1. 메인 스레드에서 최신 프레임만 빠르게 획득
        {
            std::lock_guard<std::mutex> lock(g_frameMutex);
            if (g_hasNewFrame && !g_rawYuvFrame.empty()) {
                rawYuv = g_rawYuvFrame.clone();
                g_hasNewFrame = false;
            }
        }

        // 2. 획득한 데이터가 있다면 메인 스레드에서 렌더링 작업 수행
        if (!rawYuv.empty()) {
            cv::Mat bgrFrame, resized;

            // YUV -> BGR 변환 (여기서 수행하여 네트워크 스레드의 짐을 덜어줌)
            cv::cvtColor(rawYuv, bgrFrame, cv::COLOR_YUV2BGR_I420);

            // 영상 확대
            cv::resize(bgrFrame, resized, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);
            
            // 모드에 따라 컬러맵 적용
            if (displayMode == 2) {
                cv::applyColorMap(resized, resized, cv::COLORMAP_INFERNO);
            }

            cv::imshow("Thermal Real-time (Optimized)", resized);
        }

        // 3. 지연 시간 최소화를 위해 대기 시간을 1ms로 단축
        int key = cv::waitKey(1); 
        
        if (key == 27) break;
        if (key == '1') {
            displayMode = 1;
            std::cout << "> 모드 변경: Grayscale\n";
        } else if (key == '2') {
            displayMode = 2;
            std::cout << "> 모드 변경: Inferno Color\n";
        }
    }

    std::cout << "Releasing resources...\n";
    cv::destroyAllWindows();
    sdk_release();
    return 0;
}