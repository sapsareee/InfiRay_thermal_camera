// 1번 2번 회색 열화상 교체 적용 전

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <vector>
#include <string>
#include <mutex>

// --- [OpenCV 헤더 추가] ---
#include <opencv2/opencv.hpp>

// SDK 헤더의 엉성한 표준 라이브러리 참조 해결
using namespace std;

// 리눅스에는 없는 Windows 전용 매크로와 자료형들을 강제로 (가짜로) 정의
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

// SDK 헤더
#include "LinuxDef.h"
#include "InfraredTempSDK.h"

// --- [전역 변수 및 공유 자원] ---
static bool g_tempSaved = false;
static int g_frameCount = 0;

std::mutex g_frameMutex;
cv::Mat g_currentFrame;
bool g_hasNewFrame = false;

// --- [영상 데이터 콜백 함수] ---
void videoCallBack(char *pBuffer, long BufferLen, int width, int height, void *pContext) {
    // 384 * 288 * 1.5 = 165888 (YUV420 포맷 확인)
    if (BufferLen == (width * height * 3 / 2)) {
        std::lock_guard<std::mutex> lock(g_frameMutex);
        
        // 1. RAW YUV 데이터를 Mat으로 읽기
        cv::Mat yuvFrame(height * 3 / 2, width, CV_8UC1, pBuffer);
        
        // 2. YUV420 -> BGR 컬러 변환
        // 만약 색상이 반전되거나 이상하면 cv::COLOR_YUV2BGR_YV12로 변경
        cv::cvtColor(yuvFrame, g_currentFrame, cv::COLOR_YUV2BGR_I420);
        
        g_hasNewFrame = true;
    }
}

// 온도 데이터 콜백 함수
void tempCallBack(char *pBuffer, long BufferLen, void* pContext) {
    if (g_tempSaved) return; 
    g_tempSaved = true;
    std::cout << "[Temp] First temperature data received (" << BufferLen << " bytes)\n";
}

int main() {
    std::cout << "Initializing SDK and OpenCV Window...\n";
    
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
        std::cerr << "SDK creation failed!\n";
        return -1;
    }

    ChannelInfo devInfo;
    memset(&devInfo, 0, sizeof(ChannelInfo));
    strcpy(devInfo.szUserName, username);
    strcpy(devInfo.szPWD, password);
    
    const char* targetIP = "192.168.1.123";
    strcpy(devInfo.szServerName, targetIP);
    strcpy(devInfo.szIP, targetIP);
    devInfo.channel = 0;
    devInfo.wPortNum = 3000;

    if (sdk_loginDevice(pHandle, devInfo) != 0) {
        std::cerr << "Login failed!\n";
        sdk_release();
        return -1;
    }

    // 콜백 등록
    SetDeviceVideoCallBack(pHandle, videoCallBack, nullptr);
    SetTempCallBack(pHandle, tempCallBack, nullptr);

    sdk_start_url(pHandle, devInfo.szIP);

    std::cout << "Streaming started. Press 'ESC' on the image window to exit.\n";

    // --- [메인 루프: 화면 출력] ---
    while (true) {
        cv::Mat displayMat;

        {
            std::lock_guard<std::mutex> lock(g_frameMutex);
            if (g_hasNewFrame && !g_currentFrame.empty()) {
                displayMat = g_currentFrame.clone();
                g_hasNewFrame = false;
            }
        }

        if (!displayMat.empty()) {
            // 영상이 작으므로 2배 확대해서 보기
            cv::Mat resized;
            cv::resize(displayMat, resized, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);
            
            // 만약 흑백 영상에 열화상 컬러(Rainbow 등)를 입히고 싶다면 아래 주석 해제
            // cv::applyColorMap(resized, resized, cv::COLORMAP_JET);

            cv::imshow("Thermal Camera Real-time", resized);
        }

        // 30ms 대기, ESC(27) 누르면 종료
        if (cv::waitKey(30) == 27) break;
    }

    std::cout << "Closing application...\n";
    cv::destroyAllWindows();
    sdk_release();
    
    return 0;
}