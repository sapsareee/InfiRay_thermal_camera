#include <iostream>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <vector>  // 추가: SDK 헤더 내부에서 사용됨
#include <string>  // 추가: SDK 헤더 내부에서 사용됨

// OpenCV는 사용하지 않으므로 주석 처리 또는 제거합니다.
// #include <opencv2/opencv.hpp>

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

// --- [시간 측정을 위한 전역 변수] ---
auto g_lastFrameTime = std::chrono::high_resolution_clock::now();
long g_frameCount = 0;
bool g_isRunning = true;

// --- [영상 콜백: 시간 간격만 측정] ---
void videoCallBack(char *pBuffer, long BufferLen, int width, int height, void *pContext) {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_lastFrameTime).count();
    g_lastFrameTime = now;
    g_frameCount++;

    // 터미널 도배를 막기 위해 매 프레임마다 출력하거나, 10프레임마다 출력할 수 있습니다.
    // 여기서는 변화를 즉각 보기 위해 매 프레임 출력합니다.
    if (duration > 0) {
        double fps = 1000.0 / duration;
        std::cout << "[Callback] 프레임: " << g_frameCount 
                  << " | 해상도: " << width << "x" << height 
                  << " | 간격: " << duration << " ms"
                  << " | 예상 FPS: " << fps << "\n";
    }
}

// 온도 데이터 콜백
void tempCallBack(char *pBuffer, long BufferLen, void* pContext) {
    // 사용 안 함
}

int main() {
    std::cout << "Starting Network/SDK Bottleneck Debugger...\n";
    
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
    const char* targetIP = "192.168.1.123"; // 카메라 IP 확인
    strcpy(devInfo.szIP, targetIP);
    devInfo.wPortNum = 3000;

    if (sdk_loginDevice(pHandle, devInfo) != 0) {
        std::cerr << "Login Failed\n";
        sdk_release();
        return -1;
    }

    // 시간 초기화 후 콜백 등록
    g_lastFrameTime = std::chrono::high_resolution_clock::now();
    SetDeviceVideoCallBack(pHandle, videoCallBack, nullptr);
    SetTempCallBack(pHandle, tempCallBack, nullptr);
    sdk_start_url(pHandle, devInfo.szIP);

    std::cout << "\n=====================================\n";
    std::cout << " [콜백 속도 측정 중] \n";
    std::cout << " 화면 렌더링을 완전히 끄고 순수 수신 속도만 봅니다.\n";
    std::cout << " 10초 후 프로그램이 자동으로 종료됩니다.\n";
    std::cout << "=====================================\n\n";

    // cv::waitKey() 대신 메인 스레드를 살려두기 위한 대기 로직
    int waitSeconds = 10;
    while (waitSeconds > 0) {
        sleep(1);
        waitSeconds--;
    }

    std::cout << "\nClosing debugger safely...\n";
    SetDeviceVideoCallBack(pHandle, nullptr, nullptr);
    sdk_release();
    std::cout << "Done.\n";
    
    return 0;
}