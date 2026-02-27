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

static bool g_tempSaved = false;

std::mutex g_frameMutex;
cv::Mat g_currentFrame;
bool g_hasNewFrame = false;

void videoCallBack(char *pBuffer, long BufferLen, int width, int height, void *pContext) {
    if (BufferLen == (width * height * 3 / 2)) {
        std::lock_guard<std::mutex> lock(g_frameMutex);
        cv::Mat yuvFrame(height * 3 / 2, width, CV_8UC1, pBuffer);
        cv::cvtColor(yuvFrame, g_currentFrame, cv::COLOR_YUV2BGR_I420);
        g_hasNewFrame = true;
    }
}

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

    SetDeviceVideoCallBack(pHandle, videoCallBack, nullptr);
    SetTempCallBack(pHandle, tempCallBack, nullptr);

    sdk_start_url(pHandle, devInfo.szIP);

    std::cout << "\n=====================================\n";
    std::cout << " Streaming started. \n";
    std::cout << " [단축키 안내] (영상 창을 클릭하고 누르세요)\n";
    std::cout << "  1 : 흑백 (Grayscale) 모드\n";
    std::cout << "  2 : 열화상 컬러 (ColorMap) 모드\n";
    std::cout << "  ESC : 프로그램 종료\n";
    std::cout << "=====================================\n";

    int displayMode = 1; // 1: 흑백, 2: 컬러

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
            cv::Mat resized;
            cv::resize(displayMat, resized, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);
            
            // 2번을 눌렀을 때만 컬러맵을 덧씌웁니다.
            if (displayMode == 2) {
                cv::applyColorMap(resized, resized, cv::COLORMAP_INFERNO);
            }

            cv::imshow("Thermal Camera Real-time", resized);
        }

        int key = cv::waitKey(30);
        
        if (key == 27) { // ESC 키
            break;
        } else if (key == '1') {
            if (displayMode != 1) {
                displayMode = 1;
                std::cout << "> 모드 변경: 흑백 (Grayscale)\n";
            }
        } else if (key == '2') {
            if (displayMode != 2) {
                displayMode = 2;
                std::cout << "> 모드 변경: 열화상 컬러 (ColorMap)\n";
            }
        }
    }

    std::cout << "Closing application...\n";
    cv::destroyAllWindows();
    sdk_release();
    
    return 0;
}