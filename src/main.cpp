#include <Arduino.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>
#include <soc/rtc_cntl_reg.h>
#include <IotWebConf.h>
#include <IotWebConfTParameter.h>
#include <OV2640.h>
#include <ESPmDNS.h>
#include <esp_sleep.h>
#include <lookup_camera_config.h>
#include <lookup_camera_effect.h>
#include <lookup_camera_frame_size.h>
#include <lookup_camera_gainceiling.h>
#include <lookup_camera_wb_mode.h>
#include <format_duration.h>
#include <format_number.h>
#include <moustache.h>
#include <settings.h>
#include <ArduinoJson.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <read_sensors.h>
#include <read_gps.h>

//// function prototypes

/// Sensor helpers
void shutdown_gps();  // Function prototype
void wakeup_single_reading_gps();
void wakeup_gps();
void update_gps();
void update_rssi();
void update_battery();
void update_bme280();

/// Web server handlers
void handle_root();
void handle_get_config();
void handle_get_sensor_status();
void handle_sensors();
void handle_snapshot();
void handle_restart();
void handle_bedtime();
void handle_naptime();
void handle_update_GPS();

/// Task handlers
TaskHandle_t WebServerTaskHandle = NULL;
TaskHandle_t SensorTaskHandle = NULL;


////// CONSTANTS //////
//// All updates to these trigger Pod restart. ////
// Valid I/O pins:
/// 



// Default sensor settings

// Default BME280 settings
const bool DEFAULT_BME280_PRESENT = false;
const int DEFAULT_BME280_SCL_PIN = 13;
const int DEFAULT_BME280_SDA_PIN = 15;

// Default battery reader settings
const bool DEFAULT_BATTERY_READER_PRESENT = false;
const int DEFAULT_BATTERY_READER_IO_PIN = 4;

// Default GPS settings
const bool DEFAULT_GPS_PRESENT = false;
const int DEFAULT_GPS_DRX = -1; // This is Pod RX. Remember that Pod RX -> GPS TX. (DEV: purple)
const int DEFAULT_GPS_DTX = -1;
const bool DEFAULT_GPS_PWRDWN_IO_PIN_PRESENT = false;
const int DEFAULT_GPS_PWRDWN_IO_PIN = 2;

// TODO: Add any other necessary GPS defaults

// Default bedtime settings
const int DEFAULT_BEDTIME_MAX_WAIT = 60; // seconds
const int DEFAULT_NAPTIME_BUFFER = 25; // milliseconds

////// VOLATILE VARIABLES //////
//// Sensor values
/// Note: we do not need to make params (pins/availability) for BME280, battery, or GPS volatile because we require restarts after changing these settings.

// Connection
volatile bool wifi_connected = false;
volatile float rssi = 0;
// Battery
volatile int battery_level = 0;
// BME280
volatile float temperature = 0;
volatile float humidity = 0;
volatile float pressure = 0;
volatile bool bme280_available = false; // Is the BME280 available? Can be present but not available if it fails to initialize.
// GPS
volatile float latitude = 0;
volatile float longitude = 0;
volatile float altitude = 0; 
volatile bool gps_wakeup_read_requested = false; // Has a GPS reading been requested by the user? (via /update_GPS)
volatile bool gps_available = true; // Is the GPS available? Can be present but not available if it fails to initialize.
volatile bool gps_awake = true; // Is the GPS currently awake?
// Prevent race conditions:
volatile bool is_reading_gps = false; // Is the GPS currently being read?
volatile bool is_reading_sensors = false; // Are the sensors currently being read?
// Track if we just woke up from bedtime or power cycle
volatile bool initial_sensor_reading = true;

// Naptime
volatile int baseline_naptime = 500; // milliseconds, overridden by PolliOS -> /naptime endpoint


////// Sensor objects
MyGPS myGPS;
MyBME280 myBME280;

// HTML files
extern const char index_html_min_start[] asm("_binary_html_index_min_html_start");
extern const char restart_html_min_start[] asm("_binary_html_restart_min_html_start");

auto param_group_board = iotwebconf::ParameterGroup("board", "Board settings");
auto param_board = iotwebconf::Builder<iotwebconf::SelectTParameter<sizeof(camera_configs[0])>>("bt").label("Board").optionValues((const char *)&camera_configs).optionNames((const char *)&camera_configs).optionCount(sizeof(camera_configs) / sizeof(camera_configs[0])).nameLength(sizeof(camera_configs[0])).defaultValue(DEFAULT_CAMERA_CONFIG).build();

auto param_group_camera = iotwebconf::ParameterGroup("camera", "Camera settings");
auto param_frame_duration = iotwebconf::Builder<iotwebconf::UIntTParameter<unsigned long>>("fd").label("Frame duration (ms)").defaultValue(DEFAULT_FRAME_DURATION).min(10).build();
auto param_frame_size = iotwebconf::Builder<iotwebconf::SelectTParameter<sizeof(frame_sizes[0])>>("fs").label("Frame size").optionValues((const char *)&frame_sizes).optionNames((const char *)&frame_sizes).optionCount(sizeof(frame_sizes) / sizeof(frame_sizes[0])).nameLength(sizeof(frame_sizes[0])).defaultValue(DEFAULT_FRAME_SIZE).build();
auto param_jpg_quality = iotwebconf::Builder<iotwebconf::UIntTParameter<byte>>("q").label("JPG quality").defaultValue(DEFAULT_JPEG_QUALITY).min(1).max(100).build();
auto param_enable_psram = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("eps").label("Enable PSRAM if available").defaultValue(DEFAULT_ENABLE_PSRAM).build();
auto param_frame_buffers = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("fb").label("Buffers").defaultValue(DEFAULT_BUFFERS).min(1).max(4).build();
auto param_brightness = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("b").label("Brightness").defaultValue(DEFAULT_BRIGHTNESS).min(-2).max(2).build();
auto param_contrast = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("c").label("Contrast").defaultValue(DEFAULT_CONTRAST).min(-2).max(2).build();
auto param_saturation = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("s").label("Saturation").defaultValue(DEFAULT_SATURATION).min(-2).max(2).build();
auto param_special_effect = iotwebconf::Builder<iotwebconf::SelectTParameter<sizeof(camera_effects[0])>>("e").label("Effect").optionValues((const char *)&camera_effects).optionNames((const char *)&camera_effects).optionCount(sizeof(camera_effects) / sizeof(camera_effects[0])).nameLength(sizeof(camera_effects[0])).defaultValue(DEFAULT_EFFECT).build();
auto param_whitebal = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("wb").label("White balance").defaultValue(DEFAULT_WHITE_BALANCE).build();
auto param_awb_gain = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("awbg").label("Automatic white balance gain").defaultValue(DEFAULT_WHITE_BALANCE_GAIN).build();
auto param_wb_mode = iotwebconf::Builder<iotwebconf::SelectTParameter<sizeof(camera_wb_modes[0])>>("wbm").label("White balance mode").optionValues((const char *)&camera_wb_modes).optionNames((const char *)&camera_wb_modes).optionCount(sizeof(camera_wb_modes) / sizeof(camera_wb_modes[0])).nameLength(sizeof(camera_wb_modes[0])).defaultValue(DEFAULT_WHITE_BALANCE_MODE).build();
auto param_exposure_ctrl = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("ec").label("Exposure control").defaultValue(DEFAULT_EXPOSURE_CONTROL).build();
auto param_aec2 = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("aec2").label("Auto exposure (dsp)").defaultValue(DEFAULT_AEC2).build();
auto param_ae_level = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("ael").label("Auto Exposure level").defaultValue(DEFAULT_AE_LEVEL).min(-2).max(2).build();
auto param_aec_value = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("aecv").label("Manual exposure value").defaultValue(DEFAULT_AEC_VALUE).min(9).max(1200).build();
auto param_gain_ctrl = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("gc").label("Gain control").defaultValue(DEFAULT_GAIN_CONTROL).build();
auto param_agc_gain = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("agcg").label("AGC gain").defaultValue(DEFAULT_AGC_GAIN).min(0).max(30).build();
auto param_gain_ceiling = iotwebconf::Builder<iotwebconf::SelectTParameter<sizeof(camera_gain_ceilings[0])>>("gcl").label("Auto Gain ceiling").optionValues((const char *)&camera_gain_ceilings).optionNames((const char *)&camera_gain_ceilings).optionCount(sizeof(camera_gain_ceilings) / sizeof(camera_gain_ceilings[0])).nameLength(sizeof(camera_gain_ceilings[0])).defaultValue(DEFAULT_GAIN_CEILING).build();
auto param_bpc = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("bpc").label("Black pixel correct").defaultValue(DEFAULT_BPC).build();
auto param_wpc = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("wpc").label("White pixel correct").defaultValue(DEFAULT_WPC).build();
auto param_raw_gma = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("rg").label("Gamma correct").defaultValue(DEFAULT_RAW_GAMMA).build();
auto param_lenc = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("lenc").label("Lens correction").defaultValue(DEFAULT_LENC).build();
auto param_hmirror = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("hm").label("Horizontal mirror").defaultValue(DEFAULT_HORIZONTAL_MIRROR).build();
auto param_vflip = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("vm").label("Vertical mirror").defaultValue(DEFAULT_VERTICAL_MIRROR).build();
auto param_dcw = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("dcw").label("Downsize enable").defaultValue(DEFAULT_DCW).build();
auto param_colorbar = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("cb").label("Colorbar").defaultValue(DEFAULT_COLORBAR).build();

auto param_group_bme280 = iotwebconf::ParameterGroup("bme280", "Weather sensor (BME280) settings");
auto param_bme280_present = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("bme280_present").label("BME280 present").defaultValue(DEFAULT_BME280_PRESENT).build();
auto param_bme280_scl_pin = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("bme280_io_pin").label("BME280 SCL Pin").defaultValue(DEFAULT_BME280_SCL_PIN).min(2).max(16).build();
auto param_bme280_sda_pin = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("bme280_sda_pin").label("BME280 SDA Pin").defaultValue(DEFAULT_BME280_SDA_PIN).min(2).max(16).build();

auto param_group_battery_reader = iotwebconf::ParameterGroup("battery_reader", "Battery Reader settings");
auto param_battery_reader_present = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("battery_reader_present").label("Battery Reader present").defaultValue(DEFAULT_BATTERY_READER_PRESENT).build();
auto param_battery_reader_io_pin = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("battery_reader_io_pin").label("Battery Reader I/O Pin").defaultValue(DEFAULT_BATTERY_READER_IO_PIN).min(2).max(16).build();

auto param_group_gps = iotwebconf::ParameterGroup("gps", "GPS (NEO-6m) settings");
auto param_gps_present = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("gps_present").label("GPS present").defaultValue(DEFAULT_GPS_PRESENT).build();
auto param_gps_drx = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("gps_drx").label("GPS dRX").defaultValue(DEFAULT_GPS_DRX).min(2).max(16).build();
auto param_gps_dtx = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("gps_dtx").label("GPS dTX").defaultValue(DEFAULT_GPS_DTX).min(2).max(16).build();
auto param_gps_pwrctl_io_pin_present = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("gps_power_control_present").label("GPS power control present").defaultValue(DEFAULT_GPS_PWRDWN_IO_PIN_PRESENT).build();
auto param_gps_pwrctl_io_pin = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("gps_power_control_io_pin").label("GPS power control I/O pin").defaultValue(DEFAULT_GPS_PWRDWN_IO_PIN).min(2).max(16).build();

auto param_group_bedtime = iotwebconf::ParameterGroup("bedtime", "Bedtime settings");
auto param_bedtime_max_wait = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("bedtime_max_wait").label("Max sensor wait time forcing bedtime(s)").defaultValue(DEFAULT_BEDTIME_MAX_WAIT).min(0).max(3000).build();

auto param_group_naptime = iotwebconf::ParameterGroup("naptime", "Naptime settings");
auto param_naptime_baseline = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("naptime_baseline").label("Baseline naptime(ms). Overidden by PolliOS.").defaultValue(DEFAULT_NAPTIME_BUFFER).min(0).max(10000).build();

// Camera
OV2640 cam;
// DNS Server
DNSServer dnsServer;
// Web server
WebServer web_server(80);

auto thingName = String(WIFI_SSID) + "-" + String(ESP.getEfuseMac(), 16);
IotWebConf iotWebConf(thingName.c_str(), &dnsServer, &web_server, WIFI_PASSWORD, CONFIG_VERSION);

// // Keep track of config changes. This will allow a reset of the device
// bool config_changed = false;
// Camera initialization result
esp_err_t camera_init_result;

void stream_text_file_gzip(const unsigned char *content, size_t length, const char *mime_type)
{
  // Cache for 86400 seconds (one day)
  web_server.sendHeader("Cache-Control", "max-age=86400");
  web_server.sendHeader("Content-encoding", "gzip");
  web_server.setContentLength(length);
  web_server.send(200, mime_type, "");
  web_server.sendContent(reinterpret_cast<const char *>(content), length);
}

void handle_root()
{
  log_v("Handle root");
  // Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
    return;


  // Print the ESP free heap to the serial terminal
  Serial.print("ESP free heap (start handle_root()): ");
  Serial.println(ESP.getFreeHeap());



  // Format hostname
  auto hostname = "PolliPod-" + WiFi.macAddress() + ".local";
  hostname.replace(":", "");
  hostname.toLowerCase();

  // Wifi Modes
  const char *wifi_modes[] = {"NULL", "STA", "AP", "STA+AP"};
  auto ipv4 = WiFi.getMode() == WIFI_MODE_AP ? WiFi.softAPIP() : WiFi.localIP();
  auto ipv6 = WiFi.getMode() == WIFI_MODE_AP ? WiFi.softAPIPv6() : WiFi.localIPv6();

  moustache_variable_t substitutions[] = {
      // // Config Changed?
      // {"ConfigChanged", String(config_changed)},
      // Version / CPU
      {"AppTitle", APP_TITLE},
      {"AppVersion", APP_VERSION},
      {"ThingName", iotWebConf.getThingName()},
      {"SDKVersion", ESP.getSdkVersion()},
      {"ChipModel", ESP.getChipModel()},
      {"ChipRevision", String(ESP.getChipRevision())},
      {"CpuFreqMHz", String(ESP.getCpuFreqMHz())},
      {"CpuCores", String(ESP.getChipCores())},
      {"FlashSize", format_memory(ESP.getFlashChipSize(), 0)},
      {"HeapSize", format_memory(ESP.getHeapSize())},
      {"PsRamSize", format_memory(ESP.getPsramSize(), 0)},
      // Diagnostics
      {"Uptime", String(format_duration(millis() / 1000))},
      {"FreeHeap", format_memory(ESP.getFreeHeap())},
      {"MaxAllocHeap", format_memory(ESP.getMaxAllocHeap())},
      // Network
      {"HostName", hostname},
      {"MacAddress", WiFi.macAddress()},
      {"AccessPoint", WiFi.SSID()},
      {"SignalStrength", String(WiFi.RSSI())},
      {"WifiMode", wifi_modes[WiFi.getMode()]},
      {"IpV4", ipv4.toString()},
      {"IpV6", ipv6.toString()},
      {"NetworkState.ApMode", String(iotWebConf.getState() == iotwebconf::NetworkState::ApMode)},
      {"NetworkState.OnLine", String(iotWebConf.getState() == iotwebconf::NetworkState::OnLine)},
      // Camera
      {"BoardType", String(param_board.value())},
      {"FrameSize", String(param_frame_size.value())},
      {"FrameDuration", String(param_frame_duration.value())},
      {"FrameFrequency", String(1000.0 / param_frame_duration.value(), 1)},
      {"JpegQuality", String(param_jpg_quality.value())},
      {"EnablePSRAM", String(param_enable_psram.value())},
      {"FrameBuffers", String(param_frame_buffers.value())},
      {"CameraInitialized", String(camera_init_result == ESP_OK)},
      {"CameraInitResult", String(camera_init_result)},
      {"CameraInitResultText", esp_err_to_name(camera_init_result)},
      // Settings
      {"Brightness", String(param_brightness.value())},
      {"Contrast", String(param_contrast.value())},
      {"Saturation", String(param_saturation.value())},
      {"SpecialEffect", String(param_special_effect.value())},
      {"WhiteBal", String(param_whitebal.value())},
      {"AwbGain", String(param_awb_gain.value())},
      {"WbMode", String(param_wb_mode.value())},
      {"ExposureCtrl", String(param_exposure_ctrl.value())},
      {"Aec2", String(param_aec2.value())},
      {"AeLevel", String(param_ae_level.value())},
      {"AecValue", String(param_aec_value.value())},
      {"GainCtrl", String(param_gain_ctrl.value())},
      {"AgcGain", String(param_agc_gain.value())},
      {"GainCeiling", String(param_gain_ceiling.value())},
      {"Bpc", String(param_bpc.value())},
      {"Wpc", String(param_wpc.value())},
      {"RawGma", String(param_raw_gma.value())},
      {"Lenc", String(param_lenc.value())},
      {"HMirror", String(param_hmirror.value())},
      {"VFlip", String(param_vflip.value())},
      {"Dcw", String(param_dcw.value())},
      {"ColorBar", String(param_colorbar.value())},
      // Weather Sensor (BME280)
      {"bme280_present", String(param_bme280_present.value())},
      {"bme280_scl_pin", String(param_bme280_scl_pin.value())},
      {"bme280_sda_pin", String(param_bme280_sda_pin.value())},
      // Battery Reader
      {"battery_reader_present", String(param_battery_reader_present.value())},
      {"battery_reader_io_pin", String(param_battery_reader_io_pin.value())},
      // GPS (NEO-6m)
      {"gps_present", String(param_gps_present.value())},
      {"gps_drx", String(param_gps_drx.value())},
      {"gps_dtx", String(param_gps_dtx.value())},
      {"gps_power_control_present", String(param_gps_pwrctl_io_pin_present.value())},
      {"gps_power_control_io_pin", String(param_gps_pwrctl_io_pin.value())},
      // Bedtime
    {"bedtime_max_wait", String(param_bedtime_max_wait.value())},
    {"naptime_baseline", String(param_naptime_baseline.value())}
  };

  Serial.print("ESP free heap (pre-render handle_root()): ");
  Serial.println(ESP.getFreeHeap());

  web_server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  auto html = moustache_render(index_html_min_start, substitutions);
  web_server.send(200, "text/html", html);
}


void handle_get_config() {
    // Create a JSON object
    DynamicJsonDocument doc(4096); // 4KB might be necessary due to the number of variables. Adjust if needed.

    doc["param_frame_duration"] = param_frame_duration.value();
    doc["param_frame_size"] = param_frame_size.value();
    doc["param_jpg_quality"] = param_jpg_quality.value();
    doc["param_enable_psram"] = param_enable_psram.value();
    doc["param_frame_buffers"] = param_frame_buffers.value();
    doc["param_brightness"] = param_brightness.value();
    doc["param_contrast"] = param_contrast.value();
    doc["param_saturation"] = param_saturation.value();
    doc["param_special_effect"] = param_special_effect.value();
    doc["param_whitebal"] = param_whitebal.value();
    doc["param_awb_gain"] = param_awb_gain.value();
    doc["param_wb_mode"] = param_wb_mode.value();
    doc["param_exposure_ctrl"] = param_exposure_ctrl.value();
    doc["param_aec2"] = param_aec2.value();
    doc["param_ae_level"] = param_ae_level.value();
    doc["param_aec_value"] = param_aec_value.value();
    doc["param_gain_ctrl"] = param_gain_ctrl.value();
    doc["param_agc_gain"] = param_agc_gain.value();
    doc["param_gain_ceiling"] = param_gain_ceiling.value();
    doc["param_bpc"] = param_bpc.value();
    doc["param_wpc"] = param_wpc.value();
    doc["param_raw_gma"] = param_raw_gma.value();
    doc["param_lenc"] = param_lenc.value();
    doc["param_hmirror"] = param_hmirror.value();
    doc["param_vflip"] = param_vflip.value();
    doc["param_bme280_present"] = param_bme280_present.value();
    doc["param_bme280_scl_pin"] = param_bme280_scl_pin.value();
    doc["param_bme280_sda_pin"] = param_bme280_sda_pin.value();
    doc["param_battery_reader_io_pin"] = param_battery_reader_io_pin.value();
    doc["param_gps_present"] = param_gps_present.value();
    doc["param_gps_drx"] = param_gps_drx.value();
    doc["param_gps_dtx"] = param_gps_dtx.value();
    doc["param_gps_pwrctl_io_pin_present"] = param_gps_pwrctl_io_pin_present.value();
    doc["param_gps_pwrctl_io_pin"] = param_gps_pwrctl_io_pin.value();
    doc["param_bedtime_max_wait"] = param_bedtime_max_wait.value();
    doc["param_naptime_baseline"] = param_naptime_baseline.value();

    doc["bme280_available"] = bme280_available;
    doc["gps_available"] = gps_available;
    doc["param_battery_reader_present"] = param_battery_reader_present.value();
    doc["gps_awake"] = gps_awake;
    doc["is_reading_gps"] = is_reading_gps;
    doc["is_reading_sensors"] = is_reading_sensors;
    doc["initial_sensor_reading"] = initial_sensor_reading;
    doc["rssi"] = rssi;
    doc["battery_level"] = battery_level;

    String output;
    serializeJson(doc, output);
    web_server.send(200, "application/json", output);
}

void handle_get_sensor_status() {
    // Create a JSON object
    DynamicJsonDocument doc(2048); // Might be able to reduce this size. Adjust if needed.

    doc["bme280_available"] = bme280_available;
    doc["gps_available"] = gps_available;
    doc["gps_awake"] = gps_awake;
    doc["is_reading_gps"] = is_reading_gps;
    doc["is_reading_sensors"] = is_reading_sensors;
    doc["initial_sensor_reading"] = initial_sensor_reading;
    doc["rssi"] = rssi;
    doc["battery_reader_present"] = param_battery_reader_present.value();
    doc["battery_level"] = battery_level;

    String output;
    serializeJson(doc, output);
    web_server.send(200, "application/json", output);
}

void handle_restart()
{
  log_v("Handle restart");

  // Comment out authentication check
  /* 
  if (!web_server.authenticate("admin", iotWebConf.getApPasswordParameter()->valueBuffer))
  {
    web_server.requestAuthentication(BASIC_AUTH, APP_TITLE, "401 Unauthorized<br><br>The password is incorrect.");
    return;
  }
  */

  moustache_variable_t substitutions[] = {
      {"AppTitle", APP_TITLE},
      {"AppVersion", APP_VERSION},
      {"ThingName", iotWebConf.getThingName()}};

  auto html = moustache_render(restart_html_min_start, substitutions);
  web_server.send(200, "text/html", html);
  log_v("Restarting... Press refresh to connect again");
  sleep(100);
  ESP.restart();
}

void handle_naptime() {
    //// Updates the naptime baseline value.
    /// This is used to calculate the sleep duration after a snapshot.

    // Check if the client has sent a 'naptime' parameter in the request
    if (web_server.hasArg("naptime")) {
        String naptimeValue = web_server.arg("naptime");

        // Convert the received value to an integer
        int newNaptime = naptimeValue.toInt();

        // Update the global variable if the value is valid (i.e., greater than zero)
        if (newNaptime > 0) {
            baseline_naptime = newNaptime;
            param_naptime_baseline.value() = newNaptime; // CLARIFY: Does this value persist?

            // Respond to the client indicating the updated naptime value
            web_server.send(200, "text/plain", "Naptime updated to " + String(baseline_naptime) + " milliseconds.");
            return;
        }
    }

    // If the parameter was missing or invalid, send an error response
    web_server.send(400, "text/plain", "Invalid naptime value provided.");
}

void handle_bedtime() {
    // Extract sleep duration from the request
    int sleep_minutes = web_server.arg("duration").toInt();

    // Check if is_reading_sensors is True
    uint32_t startTime = millis();
    while (is_reading_sensors) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait for 100ms
        if (millis() - startTime >= param_bedtime_max_wait.value() * 1000) {
            // Exceeded maximum waiting time, so break out of the loop
            break;
        }
    }

    // If param_gps_pwrctl_io_pin_present is True, shut down GPS
    if (param_gps_pwrctl_io_pin_present.value()) {
        shutdown_gps();
    }

    // Convert minutes to microseconds for esp_sleep_enable_timer_wakeup
    uint64_t sleep_time_us = sleep_minutes * 60 * 1000000ULL;

    // Configure ESP32 to wake up after sleep_time_us microseconds
    esp_sleep_enable_timer_wakeup(sleep_time_us);

    // Send a response to the client
    String message = "Going to bed for " + String(sleep_minutes) + " minutes.";
    web_server.send(200, "text/plain", message.c_str());

    // Finally, put the ESP32 into deep sleep
    esp_deep_sleep_start();
}

void handle_snapshot()
{
  // Start timing the snapshot process
  uint32_t startTime = millis();

  log_v("handle_snapshot");
  if (camera_init_result != ESP_OK)
  {
    web_server.send(404, "text/plain", "Camera is not initialized");
    return;
  }

  // Remove old images stored in the frame buffer
  auto frame_buffers = param_frame_buffers.value();
  while (frame_buffers--)
    cam.run();

  auto fb_len = cam.getSize();
  auto fb = (const char *)cam.getfb();
  if (fb == nullptr)
  {
    web_server.send(404, "text/plain", "Unable to obtain frame buffer from the camera");
    return;
  }

  web_server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  web_server.setContentLength(fb_len);
  web_server.send(200, "image/jpeg", "");
  web_server.sendContent(fb, fb_len);

  // End timing the snapshot process
  uint32_t endTime = millis();
  uint32_t elapsedTime = endTime - startTime;

  // Calculate the naptime based on baseline naptime, elapsed time, and a buffer
  int sleepDuration = baseline_naptime - elapsedTime - DEFAULT_NAPTIME_BUFFER;  // Subtract 25ms buffer

  // Check that is_reading_GPS and is_reading_sensors are false
  if (!is_reading_gps && !is_reading_sensors)
  {
    // Ensure sleep duration is positive
    if (sleepDuration > 0)
    {
      // Enable timer wakeup for the calculated sleep duration
      esp_sleep_enable_timer_wakeup(sleepDuration * 1000);  // microseconds
      esp_light_sleep_start();  // Enter light sleep mode
    }
  }
}

void handle_sensors()
{

  //// Triggers task2 to read sensors and update global variables. ////
  // Trigger SensorTaskHandle to update sensor values
  xTaskNotify(SensorTaskHandle, 0, eNoAction); // This sends a notification to SensorTaskHandle to update sensor values

  // Wait for 25ms to give the sensor task some time to update the global values
  vTaskDelay(pdMS_TO_TICKS(25));  

  // Create a JSON document to package the sensor values
  DynamicJsonDocument doc(256);

  // Add the sensor values to the JSON document
  doc["rssi"] = rssi;
  doc["battery_level"] = battery_level;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["pressure"] = pressure;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;
  doc["altitude"] = altitude;

  // Serialize the JSON document
  String jsonOutput;
  serializeJson(doc, jsonOutput);

  // Send the JSON data using the web_server
  web_server.send(200, "application/json", jsonOutput);
}


void handle_update_GPS()
{
  //// Triggers task2, but asks for wakeup read. ////
  if (!param_gps_present.value()) {
    web_server.send(404, "text/plain", "GPS is not present.");
    return;
  }
  if (!param_gps_pwrctl_io_pin_present.value()) {
    web_server.send(404, "text/plain", "GPS power control is not present. Just call /sensors instead.");
    return;
  }

  // Trigger task2 to read GPS and update global variables.
  gps_wakeup_read_requested = true;

  // Trigger the task to run by sending it a notification.
  if (SensorTaskHandle != NULL) {
    xTaskNotify(SensorTaskHandle, 0, eNoAction); // The '0' and 'eNoAction' mean no specific action is tied to this notification. 
                                                 // It's just a signal to wake up the task.
  }

  web_server.send(200, "text/plain", "GPS reading requested.");
}


////// TASK 1: web_server //////
/// COMMENT: TASK 1
void web_server_task(void* parameter) {

  log_v("Task 1: Beginning Task 1 (web_server) on core 0");
    
  log_v("Task 1: Starting web server");
  // Set up URL handlers on the web server
  web_server.on("/", HTTP_GET, handle_root);
  web_server.on("/config", [] { iotWebConf.handleConfig(); });
  web_server.on("/restart", HTTP_GET, handle_restart);
  web_server.on("/snapshot", HTTP_GET, handle_snapshot);
  web_server.on("/get_config", HTTP_GET, handle_get_config);
  web_server.on("/get_sensor_status", HTTP_GET, handle_get_sensor_status);
  web_server.on("/sensors", HTTP_GET, handle_sensors);
  web_server.on("/update_GPS", HTTP_GET, handle_update_GPS);
  web_server.on("/naptime", HTTP_POST, handle_naptime);
  web_server.on("/bedtime", HTTP_POST, handle_bedtime);
  web_server.on("/shutdown_GPS", HTTP_POST, shutdown_gps);
  web_server.on("/wakeup_GPS", HTTP_POST, wakeup_gps);
  web_server.onNotFound([]() { iotWebConf.handleNotFound(); });
  log_v("Task 1: Web server handlers set up");

  for(;;) {

    if (initial_sensor_reading && wifi_connected) {
      // Read sensors and update global variables
      // VERIFY: May want to review this if I2C on ADC2 causes WiFi to drop!
      log_v("Task 1: Initial sensor reading triggered.");
      xTaskNotify(SensorTaskHandle, 0, eNoAction);
      initial_sensor_reading = false;
    }

    iotWebConf.doLoop();

    // A delay to prevent the task from consuming unnecessary CPU
    vTaskDelay(pdMS_TO_TICKS(10));  
  }
}



////// TASK 2: update_sensor_values_task //////
/// COMMENT: TASK 2

void update_sensor_values_task(void* parameter) {

  log_v("Task 2: Beginning Task 2 (update_sensor_values_task) on core 1. Will wait for notification.");

  uint32_t notificationValue;
  const TickType_t xMaxBlockTime = portMAX_DELAY; // Wait indefinitely for a notification

  for(;;) {
    // Wait for a notification
    if (xTaskNotifyWait(0, 0, &notificationValue, xMaxBlockTime) == pdTRUE) {
      log_v("Task 2 (update_sensor_values_task): received notification.");
      // If notified, process sensor readings:

      // Set the global flag to indicate that we are reading the sensors
      is_reading_sensors = true;

      // Check if BME280 is available and update
      if (bme280_available) {
        log_v("Task 2: Updating BME280");
        update_bme280();
      }

      // Check if battery reader is present and update
      if (param_battery_reader_present.value()) {
        log_v("Task 2: Updating battery");
        update_battery();
      }

      // Update RSSI
      if (wifi_connected) {
        log_v("Task 2: Updating RSSI");
        update_rssi();
      }

      // Check if GPS is present and update
      if (param_gps_present.value() && !gps_wakeup_read_requested) {
        log_v("Task 2: Updating GPS");
        update_gps();
      }

      // Conditionally wake up GPS and update, if power-controlled and /update_GPS has been called
      if (param_gps_present.value()&& (param_gps_pwrctl_io_pin_present.value() && gps_wakeup_read_requested)) {
        log_v("Task 2: Waking up GPS and updating");
        wakeup_single_reading_gps();
        gps_wakeup_read_requested = false;
      }

      // Clear the global flag to indicate that we are no longer reading the sensors
      is_reading_sensors = false;
    }
  }
}

void update_rssi() {
    // Connection
    rssi = WiFi.RSSI();
}

void update_bme280() {
    // BME280
    temperature = myBME280.getTemperature();
    humidity = myBME280.getHumidity();
    pressure = myBME280.getPressure();
}

void update_battery() {
    // Battery
    battery_level = read_battery_voltage(param_battery_reader_io_pin.value());
}

void update_gps() {
    if (!gps_available) {
        Serial.println("GPS is not available.");
        return;
    }
    if (is_reading_gps) {
        Serial.println("GPS is already being read.");
        return;
    }
    if (!gps_awake) {
        Serial.println("GPS is asleep. wakeup read not requested, not waking up.");
    }

    // Set the global flag to indicate that we are reading the GPS
    is_reading_gps = true;

    myGPS.readFix(); // Update current fix inside the MyGPS instance
    
    // Only update the global values if we have a valid GPS fix
    if (myGPS.getLatitude() != 0 && myGPS.getLongitude() != 0) {
        // GPS
        latitude = myGPS.getLatitude();
        longitude = myGPS.getLongitude();
        altitude = myGPS.getAltitude();
    } else {
        Serial.println("No valid GPS fix. Values not updated.");
    }

    // Clear the global flag to indicate that we are no longer reading the GPS
    is_reading_gps = false;
}


// Function to shut down the GPS by turning off its power supply using the NPN transistor.
void shutdown_gps() {
    if (param_gps_pwrctl_io_pin_present.value()) {
        Serial.println("Shutting down GPS with pin: " + String(param_gps_pwrctl_io_pin.value()));
        digitalWrite(param_gps_pwrctl_io_pin.value(), LOW); // Turn off the NPN transistor
        Serial.println("GPS has been shut down.");
    } else {
        Serial.println("GPS power management not available.");
    }
    gps_awake = false;
}

// Function to wake up the GPS by turning on its power supply.
void wakeup_gps() {
    if (param_gps_pwrctl_io_pin_present.value()) {
        digitalWrite(param_gps_pwrctl_io_pin.value(), HIGH); // Turn on the NPN transistor
        Serial.println("GPS is waking up...");
        delay(2000); // A brief delay to allow the GPS some initial setup time. 
    } else {
        Serial.println("GPS power management not available.");
    }
    gps_awake = true;
}

// Function to wake up the GPS, read a single fix, and then shut it down again.
void wakeup_single_reading_gps() {
    if (!param_gps_pwrctl_io_pin_present.value()) {
        Serial.println("GPS power management not available.");
        return;
    }
    if (is_reading_gps) {
        Serial.println("GPS is already being read.");
        return;
    }
    if (gps_awake) {
        update_gps();
        shutdown_gps();
    }
    if (!gps_available) {
        Serial.println("GPS is not available.");
        return;
    }

    // Set the global flag to indicate that we are reading the GPS
    is_reading_gps = true;
    
    wakeup_gps();
    
    bool gotFix = false;
    uint32_t timeout = millis() + 60000; // We will wait for a maximum of 60 seconds for a GPS fix.

    while (!gotFix && millis() < timeout) {
        myGPS.readFix(); // Update the current fix in the MyGPS object

        // Check if the read data is a valid location fix
        if (myGPS.getLatitude() != 0 && myGPS.getLongitude() != 0) {
            // Update the global volatile variables
            latitude = myGPS.getLatitude();
            longitude = myGPS.getLongitude();
            altitude = myGPS.getAltitude();
            // STRETCH: Would be neater if this called update_gps() instead of duplicating the code.
            
            gotFix = true;
        } else {
            delay(1000);  // Delay for 1 second before trying again.
        }
    }

    if (!gotFix) {
        Serial.println("Failed to acquire GPS fix within the allotted time.");
    }

    shutdown_gps();

    // Clear the global flag to indicate that we are no longer reading the GPS
    is_reading_gps = false;
}

esp_err_t initialize_camera()
{
  log_v("initialize_camera");
  log_i("Camera config: %s", param_board.value());
  auto camera_config_template = lookup_camera_config(param_board.value());
  // Copy the settings
  camera_config_t camera_config;
  memset(&camera_config, 0, sizeof(camera_config_t));
  memcpy(&camera_config, &camera_config_template, sizeof(camera_config_t));
  log_i("Frame size: %s", param_frame_size.value());
  auto frame_size = lookup_frame_size(param_frame_size.value());
  log_i("JPEG quality: %d", param_jpg_quality.value());
  log_i("Frame duration: %d ms", param_frame_duration.value());
  camera_config.frame_size = frame_size;
  camera_config.jpeg_quality = param_jpg_quality.value();
  camera_config.grab_mode = CAMERA_GRAB_LATEST;
  log_i("Enable PSRAM: %d", param_enable_psram.value());
  log_i("Frame buffers: %d", param_frame_buffers.value());
  camera_config.fb_count = param_frame_buffers.value();

  if (param_enable_psram.value() && psramFound())
  {
    camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    log_i("PSRAM enabled!");
  }
  else
  {
    camera_config.fb_location = CAMERA_FB_IN_DRAM;
    log_i("PSRAM disabled");
  }

  return cam.init(camera_config);
}

void update_camera_settings()
{
  auto camera = esp_camera_sensor_get();
  if (camera == nullptr)
  {
    log_e("Unable to get camera sensor");
    return;
  }

  camera->set_brightness(camera, param_brightness.value());
  camera->set_contrast(camera, param_contrast.value());
  camera->set_saturation(camera, param_saturation.value());
  camera->set_special_effect(camera, lookup_camera_effect(param_special_effect.value()));
  camera->set_whitebal(camera, param_whitebal.value());
  camera->set_awb_gain(camera, param_awb_gain.value());
  camera->set_wb_mode(camera, lookup_camera_wb_mode(param_wb_mode.value()));
  camera->set_exposure_ctrl(camera, param_exposure_ctrl.value());
  camera->set_aec2(camera, param_aec2.value());
  camera->set_ae_level(camera, param_ae_level.value());
  camera->set_aec_value(camera, param_aec_value.value());
  camera->set_gain_ctrl(camera, param_gain_ctrl.value());
  camera->set_agc_gain(camera, param_agc_gain.value());
  camera->set_gainceiling(camera, lookup_camera_gainceiling(param_gain_ceiling.value()));
  camera->set_bpc(camera, param_bpc.value());
  camera->set_wpc(camera, param_wpc.value());
  camera->set_raw_gma(camera, param_raw_gma.value());
  camera->set_lenc(camera, param_lenc.value());
  camera->set_hmirror(camera, param_hmirror.value());
  camera->set_vflip(camera, param_vflip.value());
  camera->set_dcw(camera, param_dcw.value());
  camera->set_colorbar(camera, param_colorbar.value());
}

void on_connected()
{
  log_v("on_connected");
  // Start (OTA) Over The Air programming when connected
  // ArduinoOTA.begin();
  wifi_connected = true;
}

void on_config_saved()
{
  log_v("on_config_saved");
  // Set flash led intensity
  // analogWrite(LED_FLASH, 100); // Disable, as we need IO4 for sensors
  // Update camera setting
  update_camera_settings();
  handle_restart();
  // config_changed = true;
}

void setup() {
    Serial.begin(9600);

  // Disable brownout
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // LED_BUILTIN (GPIO33) has inverted logic false => LED on
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, false);

  // pinMode(LED_FLASH, OUTPUT);  // Disable, as we need IO4 for sensors
  // Turn flash led off
  // analogWrite(LED_FLASH, 0);  // Disable, as we need IO4 for sensors

  #ifdef CORE_DEBUG_LEVEL
    Serial.begin(115200);
    Serial.setDebugOutput(true);
  #endif

  log_i("CPU Freq: %d Mhz", getCpuFrequencyMhz());
  log_i("Free heap: %d bytes", ESP.getFreeHeap());
  log_i("SDK version: %s", ESP.getSdkVersion());
  log_i("Starting " APP_TITLE "...");

  if (psramFound())
  {
    psramInit();
    log_v("PSRAM found and initialized");
  }

  param_group_board.addItem(&param_board);
  iotWebConf.addParameterGroup(&param_group_board);

  param_group_camera.addItem(&param_frame_duration);
  param_group_camera.addItem(&param_frame_size);
  param_group_camera.addItem(&param_jpg_quality);
  param_group_camera.addItem(&param_enable_psram);
  param_group_camera.addItem(&param_frame_buffers);
  param_group_camera.addItem(&param_brightness);
  param_group_camera.addItem(&param_contrast);
  param_group_camera.addItem(&param_saturation);
  param_group_camera.addItem(&param_special_effect);
  param_group_camera.addItem(&param_whitebal);
  param_group_camera.addItem(&param_awb_gain);
  param_group_camera.addItem(&param_wb_mode);
  param_group_camera.addItem(&param_exposure_ctrl);
  param_group_camera.addItem(&param_aec2);
  param_group_camera.addItem(&param_ae_level);
  param_group_camera.addItem(&param_aec_value);
  param_group_camera.addItem(&param_gain_ctrl);
  param_group_camera.addItem(&param_agc_gain);
  param_group_camera.addItem(&param_gain_ceiling);
  param_group_camera.addItem(&param_bpc);
  param_group_camera.addItem(&param_wpc);
  param_group_camera.addItem(&param_raw_gma);
  param_group_camera.addItem(&param_lenc);
  param_group_camera.addItem(&param_hmirror);
  param_group_camera.addItem(&param_vflip);
  param_group_camera.addItem(&param_dcw);
  param_group_camera.addItem(&param_colorbar);
  iotWebConf.addParameterGroup(&param_group_camera);

  // BME280 group items
  param_group_bme280.addItem(&param_bme280_present);
  param_group_bme280.addItem(&param_bme280_scl_pin);
  param_group_bme280.addItem(&param_bme280_sda_pin);
  iotWebConf.addParameterGroup(&param_group_bme280);

  // Battery Reader group items
  param_group_battery_reader.addItem(&param_battery_reader_present);
  param_group_battery_reader.addItem(&param_battery_reader_io_pin);
  iotWebConf.addParameterGroup(&param_group_battery_reader);

  // GPS group items
  param_group_gps.addItem(&param_gps_present);
  param_group_gps.addItem(&param_gps_drx);
  param_group_gps.addItem(&param_gps_dtx);
  param_group_gps.addItem(&param_gps_pwrctl_io_pin_present);
  param_group_gps.addItem(&param_gps_pwrctl_io_pin);
  iotWebConf.addParameterGroup(&param_group_gps);

  // Bedtime group items
  param_group_bedtime.addItem(&param_bedtime_max_wait);
  iotWebConf.addParameterGroup(&param_group_bedtime);

  // Naptime group items
  param_group_naptime.addItem(&param_naptime_baseline);
  iotWebConf.addParameterGroup(&param_group_naptime);

  iotWebConf.getApTimeoutParameter()->visible = true;
  iotWebConf.setConfigSavedCallback(on_config_saved);
  iotWebConf.setWifiConnectionCallback(on_connected);
  iotWebConf.setStatusPin(LED_BUILTIN, LOW);
  iotWebConf.init();

  // Make this conditional on gps being present AND gps pwr ctl pin being present
  if (param_gps_present.value() && param_gps_pwrctl_io_pin_present.value()) {
      pinMode(param_gps_pwrctl_io_pin.value(), OUTPUT);
  }

  camera_init_result = initialize_camera();
  if (camera_init_result == ESP_OK)
      update_camera_settings();
  else
      log_e("Failed to initialize camera: 0x%0x. Type: %s, frame size: %s, frame rate: %d ms, jpeg quality: %d", 
      camera_init_result, param_board.value(), param_frame_size.value(), param_frame_duration.value(), param_jpg_quality.value());

  ///// Check sensors and initialize if present
  //// We initialize global sensors with default constructors, then call init method here with actual pins if present.
  /// Check if BME280 is present, initialize if so
  if (param_bme280_present.value()) {
      log_v("BME280 present, initializing...");
      log_v("SDA pin: %d", param_bme280_sda_pin.value());
      log_v("SCL pin: %d", param_bme280_scl_pin.value());
      myBME280.setPins(param_bme280_sda_pin.value(), param_bme280_scl_pin.value());
      if (myBME280.setup()) {  // Check if BME280 initialization succeeded
          log_v("BME280 initialized successfully.");
          bme280_available = true;
          update_bme280();
      } else {
          log_v("Failed to initialize BME280.");
          bme280_available = false;
      }
  } else {
      log_v("BME280 not present.");
      bme280_available = false;
  }

  /// Setup GPS and get initial read. Update global variable (gps_awake).
  // If power-controlled, turn off GPS after initial read, and set gps_awake to false.
  if (param_gps_present.value()) {
      log_v("GPS present, initializing...");
      log_v("DRX pin: %d", param_gps_drx.value());
      log_v("DTX pin: %d", param_gps_dtx.value());
      myGPS.init(param_gps_drx.value(), param_gps_dtx.value());
      if (myGPS.setup()) {  // Check if GPS initialization succeeded
          log_v("GPS initialized successfully.");
          gps_available = true;
          update_gps(); // Attempt to get initial GPS fix. Skips if not available.
          if (param_gps_pwrctl_io_pin_present.value()) {
              shutdown_gps();
          } else {
              gps_awake = true;
          }
      } else {
          log_v("Failed to initialize GPS.");
          gps_available = false;
          gps_awake = true; // Can be awake but unavailable
      }
  } else {
      log_v("GPS not present.");
      gps_available = false;
      gps_awake = false;
  }

  if (param_battery_reader_present.value()) {
    log_i("Battery level monitor is present.");
    log_i("Assigned I/O pin: %d", param_battery_reader_io_pin.value());
  } else {
    log_i("Battery level monitor is not present.");
  }
  

    // ArduinoOTA {
    //     .setPassword(OTA_PASSWORD)
    //     .onStart([]() { log_w("Starting OTA update: %s", ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem"); })
    //     .onEnd([]() { log_w("OTA update done!"); })
    //     .onProgress([](unsigned int progress, unsigned int total) { log_i("OTA Progress: %u%%\r", (progress / (total / 100))); })
    //     .onError([](ota_error_t error) {
    //         switch (error) {
    //             case OTA_AUTH_ERROR: log_e("OTA: Auth Failed"); break;
    //             case OTA_BEGIN_ERROR: log_e("OTA: Begin Failed"); break;
    //             case OTA_CONNECT_ERROR: log_e("OTA: Connect Failed"); break;
    //             case OTA_RECEIVE_ERROR: log_e("OTA: Receive Failed"); break;
    //             case OTA_END_ERROR: log_e("OTA: End Failed"); break;
    //             default: log_e("OTA error: %u", error);
    //         } 
    //     })};

  xTaskCreatePinnedToCore(web_server_task, "WebServerTask", 8192, NULL, 1, &WebServerTaskHandle, 0);
  xTaskCreatePinnedToCore(update_sensor_values_task, "SensorUpdateTask", 4096, NULL, 5, &SensorTaskHandle, 1);

    

    // Set flash led intensity
    // analogWrite(LED_FLASH, 100);  // Disable, as we need IO4 for sensors
}

void loop() {
  // ArduinoOTA.handle();
  yield();
}
