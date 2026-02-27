#include <iostream>
#include <cstring>
#include <unistd.h>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>

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

// ---- 프레임 더블 버퍼 ----
static std::mutex g_mtx;
static std::condition_variable g_cv;

static std::vector<uint8_t> g_yuvBuf[2];
static int g_writeIdx = 0;
static int g_readIdx  = 1;

static int g_width = 0;
static int g_height = 0;

static std::atomic<bool> g_hasNewFrame{false};
static std::atomic<bool> g_running{true};

// ---- 콜백: memcpy 1번만 + 더블버퍼 스왑 ----
void videoCallBack(char *pBuffer, long BufferLen, int width, int height, void *pContext) {
    const long expected = (long)(width * height * 3 / 2);
    if (BufferLen != expected || pBuffer == nullptr) return;

    {
        std::lock_guard<std::mutex> lk(g_mtx);

        g_width = width;
        g_height = height;

        auto &dst = g_yuvBuf[g_writeIdx];
        if ((long)dst.size() != BufferLen) dst.resize(BufferLen);

        // ★ 핵심: 여기서 딱 1번만 복사
        std::memcpy(dst.data(), pBuffer, BufferLen);

        // 쓰기/읽기 버퍼 교체 (메인은 readIdx만 읽고, 콜백은 writeIdx만 씀)
        std::swap(g_writeIdx, g_readIdx);

        g_hasNewFrame.store(true, std::memory_order_release);
    }
    g_cv.notify_one();
}

void tempCallBack(char *pBuffer, long BufferLen, void* pContext) {
    // 필요 시 구현
}

int main() {
    std::cout << "Starting Thermal App (Fast path: double-buffer + Y-plane render)\n";

    // OpenCV 내부 스레드로 인한 오버헤드/지연이 거슬리면 1로 고정해보세요.
    // (환경에 따라 도움 되기도/안 되기도 합니다)
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

    std::cout << "\n=====================================\n";
    std::cout << " [Fast Mode]\n";
    std::cout << "  - Double Buffer (no clone)\n";
    std::cout << "  - Render Y-plane only (no YUV->BGR)\n";
    std::cout << "  - 1: Gray / 2: Colormap / ESC: Exit\n";
    std::cout << "=====================================\n";

    int displayMode = 1;

    cv::namedWindow("Thermal", cv::WINDOW_NORMAL);
    // 확대가 목적이면 resize() 대신 창 크기만 키워서 스케일링을 GUI에게 맡길 수도 있어요.
    // (CPU resize 비용 절감 가능)
    cv::resizeWindow("Thermal", 1280, 1024);

    cv::Mat yResized;      // 8UC1
    cv::Mat colored;       // 8UC3

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

        // ★ Y-plane만 사용: I420에서 앞부분 height*width가 Y
        auto &buf = g_yuvBuf[localIdx];
        if ((int)buf.size() < localW * localH) continue;

        cv::Mat y(localH, localW, CV_8UC1, (void*)buf.data());

        // 선택 1) CPU resize로 2배 확대 (원하면 유지)
        cv::resize(y, yResized, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);

        if (displayMode == 1) {
            cv::imshow("Thermal", yResized);
        } else {
            cv::applyColorMap(yResized, colored, cv::COLORMAP_INFERNO);
            cv::imshow("Thermal", colored);
        }

        int key = cv::waitKey(1);
        if (key == 27) {
            g_running.store(false);
        } else if (key == '1') {
            displayMode = 1;
            std::cout << "> Gray\n";
        } else if (key == '2') {
            displayMode = 2;
            std::cout << "> Colormap\n";
        }
    }

    std::cout << "\nClosing...\n";
    SetDeviceVideoCallBack(pHandle, nullptr, nullptr);
    cv::destroyAllWindows();
    sdk_release();
    std::cout << "Done.\n";
    return 0;
}