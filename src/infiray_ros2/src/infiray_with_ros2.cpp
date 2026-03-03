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

// --- [ROS2 관련 헤더 추가] ---
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"

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
    
    uint8_t* temp_buffer = (uint8_t*)pBuffer;
    for (int ii = 0; ii < numPixels / 2; ii++) {
        g_tempBuf[ii * 2]     = (uint16_t)((temp_buffer[ii * 2] << 8)     + temp_buffer[ii * 2 + 1 + numPixels]);
        g_tempBuf[ii * 2 + 1] = (uint16_t)((temp_buffer[ii * 2 + 1] << 8) + temp_buffer[ii * 2 + numPixels]);
    }
}

int main(int argc, char** argv) {
    // ROS2 초기화
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("thermal_camera_node");

    // 파라미터 선언: 기본값은 false (화면 송출 끄기)
    node->declare_parameter("show_display", false);
    bool show_display = node->get_parameter("show_display").as_bool();

    // ROS2 Publisher 생성
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("/thermal/image", 10);
    auto temp_pub = node->create_publisher<std_msgs::msg::Float32>("/thermal/max_temp", 10);
    auto fire_pub = node->create_publisher<std_msgs::msg::Bool>("/thermal/fire_detected", 10);

    std::cout << "Starting Thermal App (ROS2 Integrated)\n";
    std::cout << "Local Display Mode: " << (show_display ? "ON" : "OFF") << "\n";

    cv::setNumThreads(1);

    int deviceType = 1; 
    char username[] = "admin";
    char password[] = "admin";

    sdk_set_type(deviceType, username, password);
    if (sdk_initialize() < 0) {
        std::cerr << "SDK Init Failed\n";
        rclcpp::shutdown();
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
        rclcpp::shutdown();
        return -1;
    }

    SetDeviceVideoCallBack(pHandle, videoCallBack, nullptr);
    SetTempCallBack(pHandle, tempCallBack, nullptr);
    sdk_start_url(pHandle, devInfo.szIP);

    if (show_display) {
        cv::namedWindow("Thermal", cv::WINDOW_NORMAL);
        cv::resizeWindow("Thermal", 1280, 1024); // 1280 1024
    }

    int displayMode = 1;
    cv::Mat yResized, displayMat;

    // ROS2가 정상 상태이고, 내부 플래그도 true일 때 루프 유지
    while (rclcpp::ok() && g_running.load()) {
        int localW = 0, localH = 0, localIdx = -1;

        {
            std::unique_lock<std::mutex> lk(g_mtx);
            g_cv.wait_for(lk, std::chrono::milliseconds(100), [] { 
                return g_hasNewFrame.load(std::memory_order_acquire) || !g_running.load() || !rclcpp::ok(); 
            });
            
            if (!g_running.load() || !rclcpp::ok()) break;
            
            if (g_hasNewFrame.load(std::memory_order_acquire)) {
                localW = g_width;
                localH = g_height;
                localIdx = g_readIdx;
                g_hasNewFrame.store(false, std::memory_order_release);
            }
        }

        if (localIdx < 0 || localW <= 0 || localH <= 0) {
            rclcpp::spin_some(node);
            continue;
        }
        
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
                        sumTemp += g_tempBuf[ty * localW + tx]; 
                        count++;
                    }
                }
                double avgRawTemp = (double)sumTemp / count;
                double calcValue = avgRawTemp;
                double divisor = 10.0;

                if (calcValue > 7300) {
                    calcValue = calcValue - 3300;
                    divisor = 15.0;
                } else {
                    calcValue = calcValue + 7000;
                    divisor = 30.0;
                }
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

        // --- [ROS2 Publish 파트] ---
        // 1. 영상 Publish
        std_msgs::msg::Header header;
        header.stamp = node->now();
        header.frame_id = "thermal_camera_frame";
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(header, "bgr8", displayMat).toImageMsg();
        image_pub->publish(*img_msg);

        // 2. 최고 온도 Publish
        std_msgs::msg::Float32 temp_msg;
        temp_msg.data = isTempValid ? celsius : 0.0;
        temp_pub->publish(temp_msg);

        // 3. 화재 감지 상태 Publish (임계값 예시: 80도 이상)
        std_msgs::msg::Bool fire_msg;
        fire_msg.data = (isTempValid && celsius > 80.0);
        fire_pub->publish(fire_msg);

        // --- [로컬 화면 송출 처리] ---
        if (show_display) {
            cv::imshow("Thermal", displayMat);
            int key = cv::waitKey(1);
            if (key == 27) { // ESC
                g_running.store(false);
            } else if (key == '1') {
                displayMode = 1;
            } else if (key == '2') {
                displayMode = 2;
            }
        }

        // ROS2 이벤트 처리
        rclcpp::spin_some(node);
    }

    std::cout << "\nClosing...\n";
    SetDeviceVideoCallBack(pHandle, nullptr, nullptr);
    SetTempCallBack(pHandle, nullptr, nullptr);
    if (show_display) {
        cv::destroyAllWindows();
    }
    sdk_release();
    rclcpp::shutdown();
    std::cout << "Done.\n";
    return 0;
}