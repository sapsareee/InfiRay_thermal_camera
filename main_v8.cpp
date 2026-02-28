#include <iostream>
#include <cstring>
#include <unistd.h>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <algorithm>
#include <cstdio> 

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

// ---- 프레임 더블 버퍼 (영상용) ----
static std::mutex g_mtx;
static std::condition_variable g_cv;
static std::vector<uint8_t> g_yuvBuf[2];
static int g_writeIdx = 0;
static int g_readIdx  = 1;
static int g_width = 0;
static int g_height = 0;
static std::atomic<bool> g_hasNewFrame{false};
static std::atomic<bool> g_running{true};

// ---- 온도 데이터 버퍼 ----
static std::mutex g_tempMtx;
static std::vector<uint16_t> g_tempBuf; 

// ---- 영상 콜백 ----
void videoCallBack(char *pBuffer, long BufferLen, int width, int height, void *pContext) {
    const long expected = (long)(width * height * 3 / 2);
    if (BufferLen != expected || pBuffer == nullptr) return;

    {
        std::lock_guard<std::mutex> lk(g_mtx);
        g_width = width;
        g_height = height;

        auto &dst = g_yuvBuf[g_writeIdx];
        if ((long)dst.size() != BufferLen) dst.resize(BufferLen);

        std::memcpy(dst.data(), pBuffer, BufferLen);
        std::swap(g_writeIdx, g_readIdx);
        g_hasNewFrame.store(true, std::memory_order_release);
    }
    g_cv.notify_one();
}

// ---- 온도 데이터 콜백 ----
void tempCallBack(char *pBuffer, long BufferLen, void* pContext) {
    if (pBuffer == nullptr || BufferLen <= 0) return;
    
    int numPixels = BufferLen / 2; 

    std::lock_guard<std::mutex> lk(g_tempMtx);
    if (g_tempBuf.size() != (size_t)numPixels) {
        g_tempBuf.resize(numPixels);
    }
    
    // B타입(DeviceType 1) 전용 비트 교차 디코딩
    uint8_t* temp_buffer = (uint8_t*)pBuffer;
    for (int ii = 0; ii < numPixels / 2; ii++) {
        g_tempBuf[ii * 2]     = (uint16_t)((temp_buffer[ii * 2] << 8)     + temp_buffer[ii * 2 + 1 + numPixels]);
        g_tempBuf[ii * 2 + 1] = (uint16_t)((temp_buffer[ii * 2 + 1] << 8) + temp_buffer[ii * 2 + numPixels]);
    }
}

int main() {
    std::cout << "Starting Thermal App (Perfect Celsius Formula Mode)\n";

    cv::setNumThreads(1);

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

    const char* targetIP = "192.168.1.123";
    strcpy(devInfo.szIP, targetIP);
    devInfo.wPortNum = 3000;

    if (sdk_loginDevice(pHandle, devInfo) != 0) {
        std::cerr << "Login Failed\n";
        sdk_release();
        return -1;
    }

    SetDeviceVideoCallBack(pHandle, videoCallBack, nullptr);
    SetTempCallBack(pHandle, tempCallBack, nullptr);
    sdk_start_url(pHandle, devInfo.szIP);

    cv::namedWindow("Thermal", cv::WINDOW_NORMAL);
    cv::resizeWindow("Thermal", 1280, 1024);

    int displayMode = 1;
    cv::Mat yResized, displayMat;

    while (g_running.load()) {
        int localW = 0, localH = 0, localIdx = -1;

        {
            std::unique_lock<std::mutex> lk(g_mtx);
            g_cv.wait(lk, [] { return g_hasNewFrame.load(std::memory_order_acquire) || !g_running.load(); });
            if (!g_running.load()) break;

            localW = g_width;
            localH = g_height;
            localIdx = g_readIdx;
            g_hasNewFrame.store(false, std::memory_order_release);
        }

        if (localIdx < 0 || localW <= 0 || localH <= 0) continue;
        auto &buf = g_yuvBuf[localIdx];
        if ((int)buf.size() < localW * localH) continue;

        cv::Mat y(localH, localW, CV_8UC1, (void*)buf.data());

        cv::Mat avgMat;
        cv::boxFilter(y, avgMat, CV_8U, cv::Size(30, 30));
        
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(avgMat, &minVal, &maxVal, &minLoc, &maxLoc);

        int rectX = std::max(0, maxLoc.x - 15);
        int rectY = std::max(0, maxLoc.y - 15);
        if (rectX + 30 > localW) rectX = localW - 30;
        if (rectY + 30 > localH) rectY = localH - 30;

        double celsius = 0.0;
        bool isTempValid = false;
        {
            std::lock_guard<std::mutex> lk(g_tempMtx);
            if (!g_tempBuf.empty() && (int)g_tempBuf.size() == localW * localH) {
                long long sumTemp = 0;
                int count = 0;
                
                for (int ty = rectY; ty < rectY + 30; ty++) {
                    for (int tx = rectX; tx < rectX + 30; tx++) {
                        // 만약 화면과 온도의 상하 위치가 안 맞는다면 이 줄 주석 해제 (int ty = (localH - 1) - ty;)
                        sumTemp += g_tempBuf[ty * localW + tx]; 
                        count++;
                    }
                }
                double avgRawTemp = (double)sumTemp / count;
                
                // [🔥 완벽하게 복원된 구간별 온도 변환 공식]
                double calcValue = avgRawTemp;
                double divisor = 10.0;

                if (calcValue > 7300) {
                    // 7301 ~ 16383 구간
                    calcValue = calcValue - 3300;
                    divisor = 15.0;
                } else {
                    // 0 ~ 7300 구간
                    calcValue = calcValue + 7000;
                    divisor = 30.0;
                }
                
                // 최종 섭씨 변환
                celsius = (calcValue / divisor) - 273.15; 
                isTempValid = true;
            }
        }

        double scale = 2.0;
        cv::resize(y, yResized, cv::Size(), scale, scale, cv::INTER_NEAREST);

        if (displayMode == 1) {
            cv::cvtColor(yResized, displayMat, cv::COLOR_GRAY2BGR);
        } else {
            cv::applyColorMap(yResized, displayMat, cv::COLORMAP_INFERNO);
        }

        cv::Rect hotZone(rectX * scale, rectY * scale, 30 * scale, 30 * scale);
        cv::rectangle(displayMat, hotZone, cv::Scalar(0, 255, 0), 2);

        char textBuf[64];
        if (isTempValid) {
            snprintf(textBuf, sizeof(textBuf), "Max: %.1f C", celsius);
        } else {
            snprintf(textBuf, sizeof(textBuf), "Wait...");
        }

        cv::Point textLoc(hotZone.x, hotZone.y - 10);
        if (textLoc.y < 20) textLoc.y = hotZone.y + hotZone.height + 25;
        
        cv::putText(displayMat, textBuf, textLoc, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        cv::imshow("Thermal", displayMat);

        int key = cv::waitKey(1);
        if (key == 27) {
            g_running.store(false);
        } else if (key == '1') {
            displayMode = 1;
        } else if (key == '2') {
            displayMode = 2;
        }
    }

    std::cout << "\nClosing...\n";
    SetDeviceVideoCallBack(pHandle, nullptr, nullptr);
    SetTempCallBack(pHandle, nullptr, nullptr);
    cv::destroyAllWindows();
    sdk_release();
    std::cout << "Done.\n";
    return 0;
}