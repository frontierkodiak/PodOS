#include <Arduino.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>
#include <soc/rtc_cntl_reg.h>
#include <IotWebConf.h>
#include <IotWebConfTParameter.h>
#include <OV2640.h>
#include <ESPmDNS.h>
//#include <rtsp_server.h>
#include <lookup_camera_config.h>
#include <lookup_camera_effect.h>
#include <lookup_camera_frame_size.h>
#include <lookup_camera_gainceiling.h>
#include <lookup_camera_wb_mode.h>
#include <format_duration.h>
#include <format_number.h>
#include <moustache.h>
#include <settings.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <read_sensors.h>
// TODO: Import GPS library


// Default peripheral settings
const bool DEFAULT_EXTERNAL_LED_PRESENT = false;
const int DEFAULT_EXTERNAL_LED_IO_PIN = 16;

// Default sensor settings

// Default BME280 settings
const bool DEFAULT_BME280_PRESENT = false;
const int DEFAULT_BME280_SCL_PIN = 13;
const int DEFAULT_BME280_SDA_PIN = 15;

// Default battery reader settings
const bool DEFAULT_BATTERY_READER_PRESENT = false;
const int DEFAULT_BATTERY_READER_IO_PIN = 12;

// Default GPS settings
const bool DEFAULT_GPS_PRESENT = false;
const int DEFAULT_GPS_DRX = 2;
const int DEFAULT_GPS_DTX = 4;
const int DEFAULT_GPS_INTERVAL = 300; // seconds. default assumes stationary Pod.
const bool DEFAULT_GPS_PWRDWN_IO_PIN_PRESENT = false;
const int DEFAULT_GPS_PWRDWN_IO_PIN = 16;

// TODO: Add any other necessary GPS defaults

// Default bedtime settings
const int DEFAULT_BEDTIME_MAX_WAIT = 2; // minutes


////// VOLATILE VARIABLES //////
//// Sensor values
/// Note: we do not need to make params (pins/availability) for BME280, battery, or GPS volatile because we require restarts after changing these settings.

// Connection
volatile float rssi = 0;
// Battery
volatile int battery_level = 0;
// BME280
volatile float temperature = 0;
volatile float humidity = 0;
volatile float pressure = 0;
// GPS
volatile float latitude = 0;
volatile float longitude = 0;
volatile float altitude = 0;


// Naptime
volatile int baseline_naptime = 500; // milliseconds, overridden by PolliOS -> /naptime endpoint



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

auto param_group_peripheral = iotwebconf::ParameterGroup("io", "peripheral settings");
auto param_led_intensity = iotwebconf::Builder<iotwebconf::UIntTParameter<byte>>("li").label("LED flash intensity (on setup complete / config updated.)").defaultValue(DEFAULT_LED_INTENSITY).min(0).max(100).build(); // this one already exists
auto param_external_led_present = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("external_led_present").label("External LED present").defaultValue(DEFAULT_EXTERNAL_LED_PRESENT).build();
auto param_external_led_io_pin = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("external_led_io_pin").label("External LED I/O Pin").defaultValue(DEFAULT_EXTERNAL_LED_IO_PIN).min(2).max(16).build();

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
auto param_gps_interval = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("gps_interval").label("Default GPS interval (secs). Can be overridden by PolliOS.").defaultValue(DEFAULT_GPS_INTERVAL).min(15).max(120).build();
auto param_gps_power_pin_present = iotwebconf::Builder<iotwebconf::CheckboxTParameter>("gps_pwrdwn_io_pin_present").label("GPS power down I/O pin present").defaultValue(DEFAULT_GPS_PWRDWN_IO_PIN_PRESENT).build();
auto param_gps_pwr_io_pin = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("gps_pwrdwn_io_pin").label("GPS power down I/O pin").defaultValue(DEFAULT_GPS_PWRDWN_IO_PIN).min(2).max(16).build();
// NOTE: /sensors returns current sensor values. /snapshot_readsensors does NOT trigger GPS activation, unless no GPS values exist yet.
// GPS interval preserved between restarts, does not need to be volatile.

auto param_group_bedtime = iotwebconf::ParameterGroup("bedtime", "Bedtime settings");
auto param_bedtime_max_wait = iotwebconf::Builder<iotwebconf::IntTParameter<int>>("bedtime_max_wait").label("Max time to wait until reading sensors finishes before forcing bedtime (mins)").defaultValue(DEFAULT_BEDTIME_MAX_WAIT).min(1).max(60).build();


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
      // LED
      {"LedIntensity", String(param_led_intensity.value())},
      {"ExternalLedPresent", String(param_external_led_present.value())},
      {"ExternalLedIOPin", String(param_external_led_io_pin.value())},
      // Weather Sensor (BME280)
      {"BME280Present", String(param_bme280_present.value())},
      {"BME280SCLPin", String(param_bme280_scl_pin.value())},
      {"BME280SDAPin", String(param_bme280_sda_pin.value())},
      // Battery Reader
      {"BatteryReaderPresent", String(param_battery_reader_present.value())},
      {"BatteryReaderIOPin", String(param_battery_reader_io_pin.value())},
      // GPS (NEO-6m)
      {"GPSPresent", String(param_gps_present.value())},
      {"GPSdRX", String(param_gps_drx.value())},
      {"GPSdTX", String(param_gps_dtx.value())},
      {"GPSInterval", String(param_gps_interval.value())},
      {"GPS_pwrdwn_io_pin_present", String(param_gps_power_pin_present.value())},
      {"GPS_pwrdwn_io_pin", String(param_gps_pwr_io_pin.value())},
      // Bedtime
      {"BedtimeMaxWait", String(param_bedtime_max_wait.value())}};

  web_server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  auto html = moustache_render(index_html_min_start, substitutions);
  web_server.send(200, "text/html", html);
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


void handle_snapshot()
{
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
}

void handle_snapshotsensors()
{

}

void update_bme280() {
    // BME280
    temperature = myBME280.getTemperature();
    humidity = myBME280.getHumidity();
    pressure = myBME280.getPressure();
}

void update_battery() {
    // Battery
    battery_level = read_battery_voltage_function();
}

void update_gps() {
    myGPS.readFix(); // Update current fix inside the MyGPS instance
    
    // GPS
    latitude = myGPS.getLatitude();
    longitude = myGPS.getLongitude();
    altitude = myGPS.getAltitude();
}

// Function to shut down the GPS by turning off its power supply using the NPN transistor.
void shutdown_gps() {
    if (param_gps_power_pin_present) {
        digitalWrite(param_gps_pwr_io_pin, LOW); // Turn off the NPN transistor
        Serial.println("GPS has been shut down.");
    } else {
        Serial.println("GPS power management not available.");
    }
}

// Function to wake up the GPS by turning on its power supply.
void wakeup_gps() {
    if (param_gps_power_pin_present) {
        digitalWrite(param_gps_pwr_io_pin, HIGH); // Turn on the NPN transistor
        Serial.println("GPS is waking up...");
        delay(2000); // A brief delay to allow the GPS some initial setup time. 
    } else {
        Serial.println("GPS power management not available.");
    }
}

// Function to wake up the GPS, read a single fix, and then shut it down again.
void wakeup_single_reading_gps() {
    if (!param_gps_power_pin_present) {
        Serial.println("GPS power management not available.");
        return;
    }
    
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
            
            Serial.println("GPS fix acquired!");
            Serial.print("Latitude: "); Serial.println(latitude, 6);
            Serial.print("Longitude: "); Serial.println(longitude, 6);
            Serial.print("Altitude: "); Serial.println(altitude, 2);
            
            gotFix = true;
        } else {
            delay(1000);  // Delay for 1 second before trying again.
        }
    }

    if (!gotFix) {
        Serial.println("Failed to acquire GPS fix within the allotted time.");
    }

    shutdown_gps();
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
  ArduinoOTA.begin();
}

void on_config_saved()
{
  log_v("on_config_saved");
  // Set flash led intensity
  analogWrite(LED_FLASH, param_led_intensity.value());
  // Update camera setting
  update_camera_settings();
  handle_restart();
  // config_changed = true;
}

void setup() {
    Serial.begin(9600);
    if (param_gps_power_pin_present) {
        pinMode(GPS_POWER_PIN, OUTPUT);
        shutdown_gps();  // Start with the GPS off.

  
  // Disable brownout
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // LED_BUILTIN (GPIO33) has inverted logic false => LED on
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, false);

  pinMode(LED_FLASH, OUTPUT);
  // Turn flash led off
  analogWrite(LED_FLASH, 0);

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

  // Peripheral group items
  param_group_peripheral.addItem(&param_led_intensity);
  param_group_peripheral.addItem(&param_external_led_present);
  param_group_peripheral.addItem(&param_external_led_io_pin);
  iotWebConf.addParameterGroup(&param_group_peripheral);

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
  param_group_gps.addItem(&param_gps_interval);
  param_group_gps.addItem(&param_gps_pwr_io_pin_present);
  param_group_gps.addItem(&param_gps_pwr_io_pin);
  iotWebConf.addParameterGroup(&param_group_gps);

  // Bedtime group items
  param_group_bedtime.addItem(&param_bedtime_max_wait);
  iotWebConf.addParameterGroup(&param_group_bedtime);

  iotWebConf.getApTimeoutParameter()->visible = true;
  iotWebConf.setConfigSavedCallback(on_config_saved);
  iotWebConf.setWifiConnectionCallback(on_connected);
  iotWebConf.setStatusPin(LED_BUILTIN, LOW);
  iotWebConf.init();

  camera_init_result = initialize_camera();
  if (camera_init_result == ESP_OK)
    update_camera_settings();
  else
    log_e("Failed to initialize camera: 0x%0x. Type: %s, frame size: %s, frame rate: %d ms, jpeg quality: %d", camera_init_result, param_board.value(), param_frame_size.value(), param_frame_duration.value(), param_jpg_quality.value());

  // Set up required URL handlers on the web server
  web_server.on("/", HTTP_GET, handle_root);
  web_server.on("/config", []
                { iotWebConf.handleConfig(); });
  web_server.on("/restart", HTTP_GET, handle_restart);
  // Camera snapshot
  web_server.on("/snapshot", HTTP_GET, handle_snapshot);
  // Camera snapshot & start reading sensors
  web_server.on("/snapshot_readsensors", HTTP_GET, handle_snapshot_readsensors);

  web_server.onNotFound([]()
                        { iotWebConf.handleNotFound(); });

  ArduinoOTA
      .setPassword(OTA_PASSWORD)
      .onStart([]()
               { log_w("Starting OTA update: %s", ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem"); })
      .onEnd([]()
             { log_w("OTA update done!"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { log_i("OTA Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
      switch (error)
      {
      case OTA_AUTH_ERROR: log_e("OTA: Auth Failed"); break;
      case OTA_BEGIN_ERROR: log_e("OTA: Begin Failed"); break;
      case OTA_CONNECT_ERROR: log_e("OTA: Connect Failed"); break;
      case OTA_RECEIVE_ERROR: log_e("OTA: Receive Failed"); break;
      case OTA_END_ERROR: log_e("OTA: End Failed"); break;
      default: log_e("OTA error: %u", error);
      } });

  // Set flash led intensity
  analogWrite(LED_FLASH, param_led_intensity.value());
}

void loop()
{
  iotWebConf.doLoop();
  ArduinoOTA.handle();

// TODO: Add web_server, read_sensor task handles

  yield();
}