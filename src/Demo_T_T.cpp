#include <iostream>
#include <fstream>
#include <cstring>
#include "IRCNetSDK.h"
#include "IRCNetSDKDef.h"

static bool g_tempSaved = false;

// 콜백 함수: 온도 데이터를 수신하고 CSV 및 TXT 파일로 저장
// IRC_NET_HANDLE 연결 식별자, IRC_NET_TEMP_INFO_CB* tempInfo 온도 정보 콜백 구조체, IRC_NET_TEMP_EXT_INFO_CB* 확장 정보 콜백 구조체, void* 사용자 데이터
// IRC_NET_TEMP_INFO_CB* tempInfo 클래스 인건가? -> 사용하는데??
void TempCallback_V2(IRC_NET_HANDLE, IRC_NET_TEMP_INFO_CB* tempInfo, IRC_NET_TEMP_EXT_INFO_CB*, void*) {
    if (g_tempSaved || !tempInfo || !tempInfo->temp) return; // 이미 저장된 경우 또는 유효하지 않은 온도 정보가 있는 경우 함수 종료
    g_tempSaved = true; // 한번만 저장

    unsigned int width = tempInfo->width;
    unsigned int height = tempInfo->height;
    unsigned short* tempArray = reinterpret_cast<unsigned short*>(tempInfo->temp); //원시 온도 배열

    // 파일 열기
    std::ofstream ofs_csv("temperature_map.csv");
    std::ofstream ofs_txt("temperature_map.txt");

    if (!ofs_csv || !ofs_txt) {
        std::cerr << "Failed to open output files\n";
        return;
    }

    // 온도 데이터 저장 루프
    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            unsigned short raw = tempArray[y * width + x];
            // raw 값을 섭씨로 변환
            float tempC = raw / 10.0f - 273.15f;  // Temperature Cal (°C)

            ofs_csv << tempC;
            ofs_txt << tempC;

            if (x != width - 1) {
                ofs_csv << ",";
                ofs_txt << " ";
            }
        }
        ofs_csv << "\n";
        ofs_txt << "\n";
    }

    // 완료 메세지
    std::cout << "[Temp] Temperature map saved to 'temperature_map.csv' and 'temperature_map.txt' (" 
              << width << "x" << height << ")\n";
}

int main() {
    IRC_NET_Init(); // Initialize the SDK

    IRC_NET_LOGIN_INFO loginInfo; // 템플릿 / 클래스의 변수 호출 -> 그런데 변수가 템플릿 형태 or key는 . 사용
    std::strcpy(loginInfo.ip, "192.168.1.123"); // 자동으로 배치된 쓰레기값 지우려고, std::strcpy 무조건 카피
    loginInfo.port = 80; // char 사용시에는 카피로 int는 대입
    std::strcpy(loginInfo.username, "admin");
    std::strcpy(loginInfo.password, "admin");

    IRC_NET_HANDLE handle;
    if (IRC_NET_Login(&loginInfo, &handle) != IRC_NET_ERROR_OK) { // &로 변수의 주소를 가져옴. 이 함수는 로그인시도로 성공하면 handle을 반환하고 실패하면 오류코드를 반환한다.
        std::cerr << "Login failed\n"; // std::cerr는 표준 오류 스트림을 나타내며, 주로 오류 메시지를 출력하는 데 사용된다
        return -1;
    }

    IRC_NET_PREVIEW_INFO previewInfo{1, IRC_NET_STREAM_MAIN, IRC_NET_FRAME_FMT_RGBA}; // 1번 채널, 메인 스트림, RGBA 프레임 포맷 여러개임 참고~
    IRC_NET_StartPreview_V2(handle, &previewInfo, nullptr, nullptr); //Start live view
    IRC_NET_StartPullTemp_V2(handle, TempCallback_V2, nullptr); //Start pulling temperature data

    std::cout << "Press Enter to exit...\n"; // Std::cout는 표준 출력 스트림을 나타내며, 주로 프로그램의 일반적인 출력을 표시하는 데 사용된다
    std::cin.get(); // 사용자가 키보드에서 Enter 키를 누를 때까지 대기한다

    IRC_NET_StopPullTemp(handle); // Stop pulling temperature data
    IRC_NET_StopPreview(handle); // Stop live view
    IRC_NET_Logout(handle); // Logout from the device
    IRC_NET_Deinit(); // Deinitialize the SDK
    return 0;
}