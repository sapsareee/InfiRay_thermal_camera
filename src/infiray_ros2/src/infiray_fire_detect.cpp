#include <iostream>
#include <cstring>
#include <unistd.h>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <algorithm>
#include <deque>
#include <cstdio> 
#include <chrono>
#include <cstdint>
#include <thread>

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

enum class VideoSource : int
{
    SDK = 0,
    DIRECT_RTSP = 1
};

// ---- 최신 프레임 버퍼 (영상용) ----
// 영상 수신 스레드는 밝기(Y) 한 프레임만 저장하고, 메인 루프는 swap으로
// 소유권을 가져간다. 수신/처리 속도가 달라도 오래된 프레임이 쌓이지 않는다.
static std::mutex g_mtx;
static std::condition_variable g_cv;
static std::vector<uint8_t> g_pendingGrayBuf;
static int g_width = 0;
static int g_height = 0;
static bool g_hasNewFrame = false;
static std::atomic<bool> g_running{true};
static std::atomic<VideoSource> g_activeVideoSource{VideoSource::SDK};

// SDK 입력 주기와 파이프라인 드롭을 확인하기 위한 누적 통계
static uint64_t g_receivedFrames = 0;
static uint64_t g_rejectedFrames = 0;
static uint64_t g_callbackIntervalCount = 0;
static uint64_t g_callbackIntervalTotalUs = 0;
static uint64_t g_callbackIntervalMaxUs = 0;
static uint64_t g_callbackWorkCount = 0;
static uint64_t g_callbackWorkTotalUs = 0;
static uint64_t g_callbackWorkMaxUs = 0;
static std::chrono::steady_clock::time_point g_lastCallbackStamp{};

// ---- 온도 데이터 버퍼 ----
static std::mutex g_tempMtx;
static std::vector<uint16_t> g_tempBuf; 

static double rawToCelsius(double raw)
{
    double calcValue = raw;
    double divisor = 10.0;

    if (calcValue > 7300.0) {
        calcValue = calcValue - 3300.0;
        divisor = 15.0;
    } else {
        calcValue = calcValue + 7000.0;
        divisor = 30.0;
    }

    return (calcValue / divisor) - 273.15;
}

struct TempTrendSample
{
    std::chrono::steady_clock::time_point stamp;
    double avgCelsius;
};

static const char* videoSourceName(VideoSource source)
{
    return source == VideoSource::DIRECT_RTSP ? "direct RTSP" : "SDK";
}

static void submitGrayFrame(
    const uint8_t* grayData,
    size_t grayLength,
    int width,
    int height,
    VideoSource source)
{
    const auto callbackStart = std::chrono::steady_clock::now();
    const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (grayData == nullptr || width <= 0 || height <= 0 || grayLength < expected) {
        std::lock_guard<std::mutex> lk(g_mtx);
        ++g_rejectedFrames;
        return;
    }

    // 두 입력이 동시에 프레임을 공급하지 않도록 선택된 소스만 허용한다.
    if (g_activeVideoSource.load(std::memory_order_acquire) != source) return;

    {
        std::lock_guard<std::mutex> lk(g_mtx);

        if (g_lastCallbackStamp != std::chrono::steady_clock::time_point{}) {
            const auto intervalUs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    callbackStart - g_lastCallbackStamp).count());
            ++g_callbackIntervalCount;
            g_callbackIntervalTotalUs += intervalUs;
            g_callbackIntervalMaxUs = std::max(g_callbackIntervalMaxUs, intervalUs);
        }
        g_lastCallbackStamp = callbackStart;

        g_width = width;
        g_height = height;
        if (g_pendingGrayBuf.size() != expected) {
            g_pendingGrayBuf.resize(expected);
        }
        std::memcpy(g_pendingGrayBuf.data(), grayData, expected);
        g_hasNewFrame = true;
        ++g_receivedFrames;

        const auto workUs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - callbackStart).count());
        ++g_callbackWorkCount;
        g_callbackWorkTotalUs += workUs;
        g_callbackWorkMaxUs = std::max(g_callbackWorkMaxUs, workUs);
    }
    g_cv.notify_one();
}

// ---- SDK 영상 콜백 (직접 RTSP 연결 실패 시 대체용) ----
void videoCallBack(char *pBuffer, long BufferLen, int width, int height, void *pContext) {
    const long expected = static_cast<long>(width) * static_cast<long>(height) * 3 / 2;
    if (pBuffer == nullptr || width <= 0 || height <= 0 || BufferLen != expected) {
        std::lock_guard<std::mutex> lk(g_mtx);
        ++g_rejectedFrames;
        return;
    }

    // SDK의 YUV420 버퍼에서 밝기(Y) 평면만 사용한다.
    submitGrayFrame(
        reinterpret_cast<const uint8_t*>(pBuffer),
        static_cast<size_t>(width) * static_cast<size_t>(height),
        width,
        height,
        VideoSource::SDK);
}

// CHyvStream을 직접 사용할 때는 SDK wrapper가 강제로 덮어쓰는 TCP 설정을
// 피할 수 있다. VideoCallBack에는 길이가 없으므로 Y 평면 크기는 해상도로 계산한다.
void directVideoCallBack(char *pBuffer, int width, int height, void *pContext) {
    submitGrayFrame(
        reinterpret_cast<const uint8_t*>(pBuffer),
        static_cast<size_t>(width) * static_cast<size_t>(height),
        width,
        height,
        VideoSource::DIRECT_RTSP);
}

void directTempCallBack(char *pBuffer, int bufferLen, void *pContext);

// ---- 온도 데이터 콜백 ----
void tempCallBack(char *pBuffer, long BufferLen, void* pContext) {
    if (pBuffer == nullptr || BufferLen <= 0 || BufferLen % 2 != 0) return;
    
    int numPixels = BufferLen / 2; 
    if (numPixels % 2 != 0) return;

    // 변환은 콜백 전용 버퍼에서 수행하고, 완성된 데이터만 짧게 잠근 뒤 전달한다.
    static thread_local std::vector<uint16_t> decodedTemp;
    decodedTemp.resize(numPixels);
    
    uint8_t* temp_buffer = (uint8_t*)pBuffer;
    for (int ii = 0; ii < numPixels / 2; ii++) {
        decodedTemp[ii * 2]     = (uint16_t)((temp_buffer[ii * 2] << 8)     + temp_buffer[ii * 2 + 1 + numPixels]);
        decodedTemp[ii * 2 + 1] = (uint16_t)((temp_buffer[ii * 2 + 1] << 8) + temp_buffer[ii * 2 + numPixels]);
    }

    {
        std::lock_guard<std::mutex> lk(g_tempMtx);
        g_tempBuf.swap(decodedTemp);
    }
}

void directTempCallBack(char *pBuffer, int bufferLen, void *pContext) {
    // 벤더 SDK의 OtherCallBackFunc도 이 버퍼를 변환 없이 TempCallBack으로 전달한다.
    tempCallBack(pBuffer, static_cast<long>(bufferLen), pContext);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("thermal_camera_node");

    const bool show_display = node->declare_parameter<bool>("show_display", false);
    const bool use_low_latency_rtsp =
        node->declare_parameter<bool>("use_low_latency_rtsp", true);
    const std::string target_ip =
        node->declare_parameter<std::string>("camera_ip", "192.168.1.123");
    const std::string camera_username =
        node->declare_parameter<std::string>("camera_username", "admin");
    const std::string camera_password =
        node->declare_parameter<std::string>("camera_password", "admin");
    const int camera_port = static_cast<int>(
        node->declare_parameter<int64_t>("camera_control_port", 3000));
    const int camera_image_fps = static_cast<int>(
        node->declare_parameter<int64_t>("camera_image_fps", 0));
    const double target_image_fps =
        node->declare_parameter<double>("target_image_fps", 10.0);
    std::string rtsp_transport =
        node->declare_parameter<std::string>("rtsp_transport", "udp");

    if (target_image_fps <= 0.0 || target_image_fps > 60.0) {
        RCLCPP_ERROR(node->get_logger(), "target_image_fps must be in (0, 60]");
        rclcpp::shutdown();
        return -1;
    }
    if (rtsp_transport != "udp" && rtsp_transport != "tcp") {
        RCLCPP_WARN(
            node->get_logger(),
            "Unsupported rtsp_transport '%s'; using udp",
            rtsp_transport.c_str());
        rtsp_transport = "udp";
    }

    // 영상은 전달 보장보다 최신성이 중요하므로 Best Effort + depth 1을 사용한다.
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>(
        "/thermal/image",
        rclcpp::SensorDataQoS().keep_last(1)
    );

    auto trend_pub = node->create_publisher<std_msgs::msg::Float32>(
        "/thermal/temperature_trend",
        rclcpp::QoS(1).reliable()  // depth=1
    );

    auto max_temp_pub = node->create_publisher<std_msgs::msg::Float32>(
        "/thermal/max_temperature",
        rclcpp::QoS(1).reliable()  // depth=1
    );

    auto fire_pub = node->create_publisher<std_msgs::msg::Bool>(
        "/thermal/fire_detected",
        rclcpp::QoS(1).reliable()  // depth=1
    );

    std::cout << "Starting Thermal App (ROS2 Integrated)\n";
    std::cout << "Local Display Mode: " << (show_display ? "ON" : "OFF") << "\n";
    std::cout << "Video Input: "
              << (use_low_latency_rtsp ? "Direct vendor RTSP" : "SDK wrapper")
              << "\n";

    cv::setNumThreads(1);

    int deviceType = 1; 
    std::vector<char> username(camera_username.begin(), camera_username.end());
    std::vector<char> password(camera_password.begin(), camera_password.end());
    username.push_back('\0');
    password.push_back('\0');

    ChannelInfo devInfo;
    memset(&devInfo, 0, sizeof(ChannelInfo));
    std::snprintf(devInfo.szUserName, sizeof(devInfo.szUserName), "%s", camera_username.c_str());
    std::snprintf(devInfo.szPWD, sizeof(devInfo.szPWD), "%s", camera_password.c_str());
    std::snprintf(devInfo.szIP, sizeof(devInfo.szIP), "%s", target_ip.c_str());
    devInfo.wPortNum = camera_port;

    IRNETHANDLE pHandle = nullptr;
    bool sdkInitialized = false;
    auto initializeFullSdk = [&]() -> bool {
        if (sdkInitialized) return true;

        sdk_set_type(deviceType, username.data(), password.data());
        if (sdk_initialize() < 0) {
            RCLCPP_ERROR(node->get_logger(), "SDK initialization failed");
            return false;
        }

        sleep(1);
        pHandle = sdk_create();
        if (pHandle == nullptr || sdk_loginDevice(pHandle, devInfo) != 0) {
            RCLCPP_ERROR(node->get_logger(), "SDK login failed");
            sdk_release();
            pHandle = nullptr;
            return false;
        }

        sdkInitialized = true;
        return true;
    };

    // 0은 카메라 설정을 건드리지 않는 안전한 기본값이다. 양수로 지정한 경우에만
    // 제어 SDK로 설정한 뒤 완전히 해제하고, 스트림 라이브러리를 별도로 초기화한다.
    if (camera_image_fps > 0) {
        if (!initializeFullSdk()) {
            rclcpp::shutdown();
            return -1;
        }

        int originalImageFps = 0;
        const int getFpsResult = sdk_get_image_framerate(pHandle, devInfo, &originalImageFps);
        if (getFpsResult == 0) {
            RCLCPP_INFO(node->get_logger(), "Camera source frame rate: %d Hz", originalImageFps);
        } else {
            RCLCPP_WARN(
                node->get_logger(),
                "Could not read camera source frame rate (%d)",
                getFpsResult);
        }

        if (getFpsResult != 0 || camera_image_fps != originalImageFps) {
            const int setFpsResult = sdk_set_image_framerate(pHandle, devInfo, camera_image_fps);
            if (setFpsResult == 0) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Camera source frame rate set to %d Hz",
                    camera_image_fps);
            } else {
                RCLCPP_WARN(
                    node->get_logger(),
                    "Could not set camera source frame rate to %d Hz (%d); continuing",
                    camera_image_fps,
                    setFpsResult);
            }
        }

        sdk_release();
        sdkInitialized = false;
        pHandle = nullptr;
        // 일부 펌웨어는 영상 설정 적용 중 스트림 서비스를 재시작한다.
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    CHyvStream* directStream = nullptr;
    bool directSdkInitialized = false;
    bool sdkStreamStarted = false;
    bool streamStarted = false;
    if (use_low_latency_rtsp) {
        const std::string rtspUrl =
            "rtsp://" + target_ip +
            "/cam/realmonitor?channel=1&subtype=0";
        CHyvStream::InitSdk();
        directSdkInitialized = true;
        directStream = CHyvStream::Create(camera_username, camera_password);
        if (directStream != nullptr) {
            const int transport = rtsp_transport == "udp" ? TRANS_UDP : TRANS_TCP;
            directStream->SetTransType(transport);
            directStream->SetDecvType(DECV_YUV);
            directStream->SetVideoCallBack(directVideoCallBack, nullptr);
            directStream->SetOtherCallBack(directTempCallBack, nullptr);
            g_activeVideoSource.store(VideoSource::DIRECT_RTSP, std::memory_order_release);

            std::vector<char> mutableUrl(rtspUrl.begin(), rtspUrl.end());
            mutableUrl.push_back('\0');
            streamStarted = directStream->Start(mutableUrl.data());
            if (streamStarted) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Direct RTSP started: transport=%s, application frame queue=1",
                    rtsp_transport.c_str());
            } else {
                directStream->Stop();
                directStream->SetVideoCallBack(nullptr, nullptr);
                directStream->SetOtherCallBack(nullptr, nullptr);
                directStream->Release();
                directStream = nullptr;
                RCLCPP_WARN(node->get_logger(), "Direct RTSP start failed; falling back to SDK");
            }
        }
    }

    if (!streamStarted) {
        if (directSdkInitialized) {
            CHyvStream::UnInitSdk();
            directSdkInitialized = false;
        }
        if (!initializeFullSdk()) {
            rclcpp::shutdown();
            return -1;
        }

        g_activeVideoSource.store(VideoSource::SDK, std::memory_order_release);
        SetDeviceVideoCallBack(pHandle, videoCallBack, nullptr);
        SetTempCallBack(pHandle, tempCallBack, nullptr);
        if (sdk_start_url(pHandle, devInfo.szIP) != 0) {
            std::cerr << "Stream Start Failed\n";
            SetDeviceVideoCallBack(pHandle, nullptr, nullptr);
            SetTempCallBack(pHandle, nullptr, nullptr);
            sdk_release();
            rclcpp::shutdown();
            return -1;
        }
        sdkStreamStarted = true;
        streamStarted = true;
        RCLCPP_WARN(node->get_logger(), "Using SDK TCP video fallback");
    }

    if (show_display) {
        cv::namedWindow("Thermal", cv::WINDOW_NORMAL);
        cv::resizeWindow("Thermal", 1280, 1024);
    }

    int displayMode = 1;
    cv::Mat displayMat;

    using namespace std::chrono;
    const auto IMAGE_FRAME_INTERVAL = duration_cast<steady_clock::duration>(
        duration<double>(1.0 / target_image_fps));
    auto next_image_tick = steady_clock::now();

    const double FIRE_THRESHOLD_C = 40.0; // 불 감지 임계 온도 (Celsius)
    const double FIRE_HOLD_SECONDS = 1.0; // 임계 온도 이상 지속 시간 (초)
    const double TREND_WINDOW_SECONDS = 3.0; // 온도 상승 추세 계산을 위한 시간 창 (초)
    const double AVG_TEMP_MIN_FOR_FIRE = 40.0; // 불로 간주하기 위한 평균 온도 최소값 (Celsius)
    const double TREND_MIN_C_PER_SEC = 0.3; // 불로 간주하기 위한 온도 상승 추세 최소값 (Celsius/초)
    // 화재 판정 논리 fireCandidate = sustainedHot && (avgHotEnough || trendRisingEnough);
    

    auto hot_above_since = steady_clock::time_point::min();
    std::deque<TempTrendSample> trend_history;

    std::vector<uint8_t> localGrayBuf;
    int localW = 0;
    int localH = 0;
    uint64_t currentFrameSequence = 0;
    uint64_t processedFrames = 0;
    uint64_t droppedFrames = 0;
    uint64_t publishedFrames = 0;
    uint64_t lastReceivedFrames = 0;
    uint64_t lastRejectedFrames = 0;
    uint64_t lastProcessedFrames = 0;
    uint64_t lastDroppedFrames = 0;
    uint64_t lastPublishedFrames = 0;
    auto lastStatsTime = steady_clock::now();
    std_msgs::msg::Header currentFrameHeader;
    currentFrameHeader.frame_id = "thermal_camera_frame";

    auto logPipelineStats = [&]() {
        const auto now = steady_clock::now();
        const double elapsed = duration<double>(now - lastStatsTime).count();
        if (elapsed < 1.0) return;

        uint64_t received = 0;
        uint64_t rejected = 0;
        uint64_t intervalCount = 0;
        uint64_t intervalTotalUs = 0;
        uint64_t intervalMaxUs = 0;
        uint64_t workCount = 0;
        uint64_t workTotalUs = 0;
        uint64_t workMaxUs = 0;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            received = g_receivedFrames;
            rejected = g_rejectedFrames;
            intervalCount = g_callbackIntervalCount;
            intervalTotalUs = g_callbackIntervalTotalUs;
            intervalMaxUs = g_callbackIntervalMaxUs;
            workCount = g_callbackWorkCount;
            workTotalUs = g_callbackWorkTotalUs;
            workMaxUs = g_callbackWorkMaxUs;
            g_callbackIntervalCount = 0;
            g_callbackIntervalTotalUs = 0;
            g_callbackIntervalMaxUs = 0;
            g_callbackWorkCount = 0;
            g_callbackWorkTotalUs = 0;
            g_callbackWorkMaxUs = 0;
        }

        const double intervalAvgMs = intervalCount > 0
            ? static_cast<double>(intervalTotalUs) / intervalCount / 1000.0 : 0.0;
        const double workAvgMs = workCount > 0
            ? static_cast<double>(workTotalUs) / workCount / 1000.0 : 0.0;

        RCLCPP_INFO(
            node->get_logger(),
            "pipeline %.1fs | source %s | input %.1f Hz | new %.1f Hz | pub %.1f Hz | "
            "drop %llu (total %llu), reject %llu | input dt %.2f/%.2f ms, copy %.3f/%.3f ms",
            elapsed,
            videoSourceName(g_activeVideoSource.load(std::memory_order_acquire)),
            static_cast<double>(received - lastReceivedFrames) / elapsed,
            static_cast<double>(processedFrames - lastProcessedFrames) / elapsed,
            static_cast<double>(publishedFrames - lastPublishedFrames) / elapsed,
            static_cast<unsigned long long>(droppedFrames - lastDroppedFrames),
            static_cast<unsigned long long>(droppedFrames),
            static_cast<unsigned long long>(rejected - lastRejectedFrames),
            intervalAvgMs,
            static_cast<double>(intervalMaxUs) / 1000.0,
            workAvgMs,
            static_cast<double>(workMaxUs) / 1000.0);

        lastReceivedFrames = received;
        lastRejectedFrames = rejected;
        lastProcessedFrames = processedFrames;
        lastDroppedFrames = droppedFrames;
        lastPublishedFrames = publishedFrames;
        lastStatsTime = now;
    };


    while (rclcpp::ok() && g_running.load()) {
        {
            std::unique_lock<std::mutex> lk(g_mtx);
            // SDK 콜백 도착 시점과 무관하게 정확히 10 Hz tick까지 기다린다.
            g_cv.wait_until(lk, next_image_tick, [] {
                return !g_running.load() || !rclcpp::ok();
            });
            
            if (!g_running.load() || !rclcpp::ok()) break;
            
            if (g_hasNewFrame) {
                const uint64_t newSequence = g_receivedFrames;
                if (currentFrameSequence > 0 && newSequence > currentFrameSequence + 1) {
                    droppedFrames += newSequence - currentFrameSequence - 1;
                }
                currentFrameSequence = newSequence;
                localW = g_width;
                localH = g_height;
                localGrayBuf.swap(g_pendingGrayBuf);
                g_hasNewFrame = false;
                ++processedFrames;
                currentFrameHeader.stamp = node->now();
            }
        }

        const auto tickNow = steady_clock::now();
        do {
            next_image_tick += IMAGE_FRAME_INTERVAL;
        } while (next_image_tick <= tickNow);

        if (localGrayBuf.empty() || localW <= 0 || localH <= 0) {
            rclcpp::spin_some(node);
            logPipelineStats();
            continue;
        }
        
        if ((int)localGrayBuf.size() < localW * localH) {
            logPipelineStats();
            continue;
        }

        cv::Mat y(localH, localW, CV_8UC1, (void*)localGrayBuf.data());

        constexpr int ROI_SIZE = 4;
        constexpr int ROI_HALF = ROI_SIZE / 2;

        cv::Mat avgMat;
        cv::boxFilter(y, avgMat, CV_8U, cv::Size(ROI_SIZE, ROI_SIZE));
        
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(avgMat, &minVal, &maxVal, &minLoc, &maxLoc);

        int rectX = std::max(0, maxLoc.x - ROI_HALF);
        int rectY = std::max(0, maxLoc.y - ROI_HALF);
        rectX = std::min(rectX, std::max(0, localW - ROI_SIZE));
        rectY = std::min(rectY, std::max(0, localH - ROI_SIZE));

        double avgCelsius = 0.0;
        double maxCelsius = 0.0;
        double trendCelsiusPerSec = 0.0;
        double holdSeconds = 0.0;
        bool isTempValid = false;
        bool fireCandidate = false;
        {
            std::lock_guard<std::mutex> lk(g_tempMtx);
            if (!g_tempBuf.empty() && (int)g_tempBuf.size() == localW * localH) {
                long long sumTemp = 0;
                uint16_t maxRawTemp = 0;
                int count = 0;
                for (int ty = rectY; ty < rectY + ROI_SIZE; ty++) {
                    for (int tx = rectX; tx < rectX + ROI_SIZE; tx++) {
                        uint16_t rawTemp = g_tempBuf[ty * localW + tx];
                        sumTemp += rawTemp;
                        if (rawTemp > maxRawTemp) {
                            maxRawTemp = rawTemp;
                        }
                        count++;
                    }
                }

                double avgRawTemp = (double)sumTemp / count;
                avgCelsius = rawToCelsius(avgRawTemp);
                maxCelsius = rawToCelsius((double)maxRawTemp);
                isTempValid = true;

                auto now = steady_clock::now();
                trend_history.push_back({now, avgCelsius});
                while (!trend_history.empty()) {
                    auto age = duration<double>(now - trend_history.front().stamp).count();
                    if (age <= TREND_WINDOW_SECONDS) {
                        break;
                    }
                    trend_history.pop_front();
                }

                if (trend_history.size() >= 2) {
                    const auto &first = trend_history.front();
                    const auto &last = trend_history.back();
                    double elapsed = duration<double>(last.stamp - first.stamp).count();
                    if (elapsed > 0.0) {
                        trendCelsiusPerSec = (last.avgCelsius - first.avgCelsius) / elapsed;
                    }
                }

                if (maxCelsius >= FIRE_THRESHOLD_C) {
                    if (hot_above_since == steady_clock::time_point::min()) {
                        hot_above_since = now;
                    }
                    holdSeconds = duration<double>(now - hot_above_since).count();
                } else {
                    hot_above_since = steady_clock::time_point::min();
                    holdSeconds = 0.0;
                }

                const bool sustainedHot = (holdSeconds >= FIRE_HOLD_SECONDS);
                const bool avgHotEnough = (avgCelsius >= AVG_TEMP_MIN_FOR_FIRE);
                const bool trendRisingEnough = (trendCelsiusPerSec >= TREND_MIN_C_PER_SEC);
                fireCandidate = sustainedHot && (avgHotEnough || trendRisingEnough);
            }
        }

        // [수정점 2] 불필요한 이미지 확대(Resize) 제거하여 데이터 전송량 감소
        if (displayMode == 1) {
            cv::cvtColor(y, displayMat, cv::COLOR_GRAY2BGR);
        } else {
            cv::applyColorMap(y, displayMat, cv::COLORMAP_INFERNO);
        }

        // 확대 비율(scale) 제거로 좌표 원복
        cv::Rect hotZone(rectX, rectY, ROI_SIZE, ROI_SIZE);
        cv::rectangle(displayMat, hotZone, cv::Scalar(0, 255, 0), 2);

        char textBuf[64];
        if (isTempValid) {
            snprintf(textBuf, sizeof(textBuf), "Peak: %.1f C", maxCelsius);
        } else {
            snprintf(textBuf, sizeof(textBuf), "Wait...");
        }

        cv::Point textLoc(hotZone.x, hotZone.y - 10);
        if (textLoc.y < 20) textLoc.y = hotZone.y + hotZone.height + 25;
        // 폰트 크기 약간 축소 (원본 해상도에 맞춤)
        cv::putText(displayMat, textBuf, textLoc, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

        // 오탐 감소용 지표 표시
        char metrics1[96];
        char metrics2[96];
        char metrics3[96];
        char metrics4[96];

        snprintf(metrics1, sizeof(metrics1), "Avg: %.1f C", avgCelsius);
        snprintf(metrics2, sizeof(metrics2), "Max: %.1f C", maxCelsius);
        snprintf(metrics3, sizeof(metrics3), "Trend: %.2f C/s", trendCelsiusPerSec);
        snprintf(metrics4, sizeof(metrics4), "Hold: %.2f s  Fire: %s", holdSeconds, fireCandidate ? "ON" : "OFF");

        const int overlayX = 10;
        const int overlayY = 18;
        const int lineGap = 16;

        cv::putText(displayMat, metrics1, cv::Point(overlayX, overlayY), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
        cv::putText(displayMat, metrics2, cv::Point(overlayX, overlayY + lineGap), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
        cv::putText(displayMat, metrics3, cv::Point(overlayX, overlayY + lineGap * 2), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
        cv::putText(displayMat, metrics4, cv::Point(overlayX, overlayY + lineGap * 3), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(fireCandidate ? 0 : 255, fireCandidate ? 255 : 255, 0), 1);

        // 이 루프 자체가 10 Hz 고정 tick이므로 영상과 상태 토픽을 같은 주기로 발행한다.
        // 새 SDK 프레임이 늦으면 마지막 영상을 재사용하되, 원래 프레임 timestamp는 유지한다.
        sensor_msgs::msg::Image::SharedPtr img_msg =
            cv_bridge::CvImage(currentFrameHeader, "bgr8", displayMat).toImageMsg();
        image_pub->publish(*img_msg);
        ++publishedFrames;

        if (isTempValid) {
            std_msgs::msg::Float32 trend_msg;
            trend_msg.data = static_cast<float>(trendCelsiusPerSec);
            trend_pub->publish(trend_msg);

            std_msgs::msg::Float32 max_temp_msg;
            max_temp_msg.data = static_cast<float>(maxCelsius);
            max_temp_pub->publish(max_temp_msg);
        }

        std_msgs::msg::Bool fire_msg;
        fire_msg.data = (isTempValid && fireCandidate);
        fire_pub->publish(fire_msg);

        if (show_display) {
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

        rclcpp::spin_some(node);
        logPipelineStats();
    }

    std::cout << "\nClosing...\n";
    g_running.store(false);
    g_cv.notify_all();

    if (directStream != nullptr) {
        // 새 콜백 진입을 막고 이미 실행 중인 콜백이 빠져나갈 시간을 준 뒤 Stop한다.
        // SDK와 CHyvStream 전역을 동시에 초기화하지 않으므로 이 순서가 안전하다.
        directStream->SetVideoCallBack(nullptr, nullptr);
        directStream->SetOtherCallBack(nullptr, nullptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        directStream->Stop();
        directStream->Release();
        directStream = nullptr;
    }

    if (sdkStreamStarted) {
        // 이 SDK는 sdk_stop_url() 안에서 CHyvStream을 해제한다. 따라서 콜백은
        // 반드시 그 전에 해제해야 하며, 순서를 바꾸면 null 객체를 역참조한다.
        SetDeviceVideoCallBack(pHandle, nullptr, nullptr);
        SetTempCallBack(pHandle, nullptr, nullptr);
        const int stopResult = sdk_stop_url(pHandle);
        if (stopResult != 0) {
            RCLCPP_WARN(node->get_logger(), "sdk_stop_url returned %d", stopResult);
        }
    }

    // RTSP 종료는 비동기이므로 전역 자원을 해제하기 전에 drain한다.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (directSdkInitialized) {
        CHyvStream::UnInitSdk();
        directSdkInitialized = false;
    }
    if (show_display) {
        cv::destroyAllWindows();
    }
    if (sdkInitialized) {
        sdk_release();
        sdkInitialized = false;
    }
    rclcpp::shutdown();
    std::cout << "Done.\n";
    return 0;
}
