#pragma once

#define APP_TITLE "PodOS"
#define APP_VERSION "1.0" // PodOS 0.3

#define WIFI_SSID "PolliPod"
#define WIFI_PASSWORD nullptr
#define CONFIG_VERSION "1.5"

// #define OTA_PASSWORD "PolliPod"

// #define RTSP_PORT 554

#define DEFAULT_CAMERA_CONFIG "AI THINKER"
#define DEFAULT_ENABLE_PSRAM psramFound()
#define DEFAULT_BUFFERS (psramFound() ? 2 : 1)
#define DEFAULT_FRAME_DURATION 100
#define DEFAULT_FRAME_SIZE (psramFound() ? "UXGA (1600x1200)" : "UXGA (1600x1200)") // "UXGA (1600x1200)" : "SVGA (800x600)")
#define DEFAULT_JPEG_QUALITY (psramFound() ? 10 : 12)

#define DEFAULT_BRIGHTNESS  0
#define DEFAULT_CONTRAST  0
#define DEFAULT_SATURATION  0
#define DEFAULT_EFFECT  "Normal"
#define DEFAULT_WHITE_BALANCE true
#define DEFAULT_WHITE_BALANCE_GAIN true
#define DEFAULT_WHITE_BALANCE_MODE "Auto"
#define DEFAULT_EXPOSURE_CONTROL true
#define DEFAULT_AEC2 true
#define DEFAULT_AE_LEVEL 0
#define DEFAULT_AEC_VALUE 300
#define DEFAULT_GAIN_CONTROL true
#define DEFAULT_AGC_GAIN 0
#define DEFAULT_GAIN_CEILING "2X"
#define DEFAULT_BPC false
#define DEFAULT_WPC true
#define DEFAULT_RAW_GAMMA true
#define DEFAULT_LENC true
#define DEFAULT_HORIZONTAL_MIRROR false
#define DEFAULT_VERTICAL_MIRROR false
#define DEFAULT_DCW true
#define DEFAULT_COLORBAR false

#define DEFAULT_LED_INTENSITY 0
