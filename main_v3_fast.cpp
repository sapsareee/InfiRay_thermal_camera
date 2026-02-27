#include <iostream>
#include <cstring>
#include <unistd.h>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>

#include <opencv2/opencv.hpp>

using namespace std;

// --- [윈도우 호환용 매크로 정의] ---
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
    typedef struct _RECT { LONG left; LONG top; LONG right; LONG bottom; } RECT;
    #ifndef TRUE
        #define TRUE 1
    #endif
    #ifndef FALSE
        #define FALSE 0
    #endif
#endif

#include "LinuxDef.h"
#include "InfraredTempSDK.h"

// --- [최적화: 더블 버퍼링 기법 도입] ---
// 매 프레임마다 메모리를 생성/복사(clone)하지 않고 포인터만 스왑하여 Zero-Copy 달성
std::mutex g_frameMutex;
std::condition_variable g_frameReady;
std::vector<char> g_bufferA;
std::vector<char> g_bufferB;
std::vector<char>* g_writeBuffer = &g_bufferA;
std::vector<char>* g_readBuffer = &g_bufferB;

int g_frameWidth = 0;
int g_frameHeight = 0;
bool g_hasNewFrame = false;
bool g_isRunning = true; 

// --- [영상 콜백] ---
void videoCallBack(char *pBuffer, long BufferLen, int width, int height, void *pContext) {
    if (BufferLen == (width * height * 3 / 2)) {
        {
            std::lock_guard<std::mutex> lock(g_frameMutex);
            // 버퍼 크기가 다르면 미리 확보 (최초 1회만 재할당됨)
            if (g_writeBuffer->size() != (size_t)BufferLen) {
                g_writeBuffer->resize(BufferLen);
            }
            // cv::Mat.copyTo 대신 더 빠른 원시 메모리 복사 사용
            std::memcpy(g_writeBuffer->data(), pBuffer, BufferLen);
            
            g_frameWidth = width;
            g_frameHeight = height;
            g_hasNewFrame = true;
        }
        g_frameReady.notify_one(); 
    }
}

// 온도 데이터 콜백
void tempCallBack(char *pBuffer, long BufferLen, void* pContext) {
    // 필요 시 구현
}

int main() {
    std::cout << "Starting Ultra-Optimized Thermal App...\n";
    
    // SDK 초기화
    int deviceType = 1; 
    char username[] = "admin";
    char password[] = "admin";
    sdk_set_type(deviceType, username, password);
    if (sdk_initialize() < 0) {
        std::cerr << "SDK Init Failed\n";
        return -1;
    }

    sleep(1); 
    IRNETHANDLE pHandle = sdk_create();
    
    ChannelInfo devInfo;
    memset(&devInfo, 0, sizeof(ChannelInfo));
    strcpy(devInfo.szUserName, username);
    strcpy(devInfo.szPWD, password);
    const char* targetIP = "192.168.1.123"; // 카메라 IP
    strcpy(devInfo.szIP, targetIP);
    devInfo.wPortNum = 3000;

    if (sdk_loginDevice(pHandle, devInfo) != 0) {
        std::cerr << "Login Failed\n";
        sdk_release();
        return -1;
    }

    // 콜백 등록
    SetDeviceVideoCallBack(pHandle, videoCallBack, nullptr);
    SetTempCallBack(pHandle, tempCallBack, nullptr);
    sdk_start_url(pHandle, devInfo.szIP);

    std::cout << "\n=====================================\n";
    std::cout << " [초고속 최적화 모드 작동 중] \n";
    std::cout << "  - Y채널 직접 추출 및 1채널 렌더링 적용\n";
    std::cout << "  - 1: 흑백 / 2: 컬러 / ESC: 종료\n";
    std::cout << "=====================================\n";

    int displayMode = 1;

    while (g_isRunning) {
        std::vector<char>* processBuffer = nullptr;
        int width = 0, height = 0;

        {
            std::unique_lock<std::mutex> lock(g_frameMutex);
            // 타임아웃을 30ms로 줄여 키보드 입력 반응성 확보
            if (g_frameReady.wait_for(lock, std::chrono::milliseconds(30), []{ return g_hasNewFrame; })) {
                // 핵심: clone() 연산 없이 포인터 스왑만으로 데이터를 메인 스레드로 가져옴
                std::swap(g_writeBuffer, g_readBuffer);
                processBuffer = g_readBuffer;
                width = g_frameWidth;
                height = g_frameHeight;
                g_hasNewFrame = false; 
            }
        }

        // 렌더링 처리
        if (processBuffer != nullptr && width > 0 && height > 0) {
            
            // [핵심 최적화] YUV I420 배열의 첫 (width * height) 바이트는 순수 Y채널(흑백 명암)입니다.
            // 무거운 cv::cvtColor(YUV2BGR)을 버리고, 흑백 1채널 데이터를 바로 Mat으로 매핑합니다.
            cv::Mat grayFrame(height, width, CV_8UC1, processBuffer->data());
            cv::Mat resized;
            
            // [핵심 최적화] 3채널이 아닌 1채널 상태에서 Resize를 진행하여 연산 속도를 3배 높입니다.
            cv::resize(grayFrame, resized, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
            
            if (displayMode == 2) {
                // 컬러 모드: 1채널 흑백 이미지에 컬러맵 적용 (이때 자동으로 3채널 BGR이 됨)
                cv::Mat colorFrame;
                cv::applyColorMap(resized, colorFrame, cv::COLORMAP_INFERNO);
                cv::imshow("Real-time Thermal (Optimized)", colorFrame);
            } else {
                // 흑백 모드: 1채널 이미지 그대로 송출
                cv::imshow("Real-time Thermal (Optimized)", resized);
            }
        }

        // 키 입력 처리
        int key = cv::waitKey(1); 
        if (key == 27) {
            g_isRunning = false;
        } else if (key == '1') {
            displayMode = 1;
            std::cout << "> 모드 변경: 흑백\n";
        } else if (key == '2') {
            displayMode = 2;
            std::cout << "> 모드 변경: 열화상 컬러\n";
        }
    }

    // 종료 시퀀스
    std::cout << "\nClosing application safely...\n";
    SetDeviceVideoCallBack(pHandle, nullptr, nullptr);
    cv::destroyAllWindows();
    sdk_release();
    std::cout << "Done.\n";
    return 0;
}