#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <LittleFS.h>
#include <HTTPClient.h>
#include <ESP32Ping.h>
#include <mbedtls/base64.h>
#include <WiFiUdp.h>
#include <Arduino_SNMP_Manager.h>
#include <regex.h>
#include <time.h>
#include <esp_random.h>
#include <ElegantOTA.h>

// MeshCore layered protocol implementation
#include "MeshCore.hpp"

#include "config.hpp"

// --- LCD and Touch Screen Support (Conditional) ---
// Define HAS_LCD=1 in build flags to enable LCD/touch support
// This is enabled for boards like the Guition ESP32-4848S040
#ifdef HAS_LCD
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

// --- Display and touch configuration for ESP32-4848S040 ---
#ifndef TFT_WIDTH
#define TFT_WIDTH 480
#endif

#ifndef TFT_HEIGHT
#define TFT_HEIGHT 480
#endif

#ifndef TFT_SCLK_PIN
#define TFT_SCLK_PIN 48  // SPI clock for ST7701 initialization
#endif

#ifndef TFT_MOSI_PIN
#define TFT_MOSI_PIN 47  // SPI data for ST7701 initialization
#endif

#ifndef TFT_MISO_PIN
#define TFT_MISO_PIN -1
#endif

#ifndef TFT_CS_PIN
#define TFT_CS_PIN 39   // LCD_CS per pinout (IO39)
#endif

#ifndef TFT_DC_PIN
#define TFT_DC_PIN 9
#endif

#ifndef TFT_RST_PIN
#define TFT_RST_PIN -1  // No hardware reset pin - uses software reset via SPI init
#endif

#ifndef TFT_BL_PIN
#define TFT_BL_PIN 38   // Backlight control pin
#endif

#ifndef TOUCH_SDA_PIN
#define TOUCH_SDA_PIN 19  // Touch I2C SDA
#endif

#ifndef TOUCH_SCL_PIN
#define TOUCH_SCL_PIN 45  // Touch I2C SCL
#endif

#ifndef TOUCH_INT_PIN
// Guition reference design does not wire GT911 INT to the ESP32-S3, so default to unused.
#define TOUCH_INT_PIN -1
#endif

#ifndef TOUCH_RST_PIN
// GT911 reset is not connected on most Guition ESP32-4848S040 revisions; leave unmanaged by default.
#define TOUCH_RST_PIN -1
#endif

// LGFX device configured for ST7701 parallel RGB (16-bit)
class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_RGB _bus_instance;
  lgfx::Panel_ST7701_guition_esp32_4848S040 _panel_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_GT911 _touch_instance;

 public:
  LGFX() {
    // Panel configuration
    {
      auto cfg = _panel_instance.config();
      cfg.memory_width  = TFT_WIDTH;
      cfg.memory_height = TFT_HEIGHT;
      cfg.panel_width   = TFT_WIDTH;
      cfg.panel_height  = TFT_HEIGHT;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.pin_rst = TFT_RST_PIN;
      _panel_instance.config(cfg);
    }

    // Panel detail configuration (SPI pins for ST7701 initialization)
    {
      auto cfg = _panel_instance.config_detail();
      cfg.pin_cs   = TFT_CS_PIN;
      cfg.pin_sclk = TFT_SCLK_PIN;
      cfg.pin_mosi = TFT_MOSI_PIN;
      _panel_instance.config_detail(cfg);
    }

    // RGB Bus configuration
    {
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;
      // Data pins d0..d15 mapped for ESP32-4848S040
      cfg.pin_d0  = GPIO_NUM_4;   // DB1(B)  -> IO4
      cfg.pin_d1  = GPIO_NUM_5;   // DB2(B)  -> IO5
      cfg.pin_d2  = GPIO_NUM_6;   // DB3(B)  -> IO6
      cfg.pin_d3  = GPIO_NUM_7;   // DB4(B)  -> IO7
      cfg.pin_d4  = GPIO_NUM_15;  // DB5(B)  -> IO15
      cfg.pin_d5  = GPIO_NUM_8;   // DB6(G)  -> IO8
      cfg.pin_d6  = GPIO_NUM_20;  // DB7(G)  -> IO20
      cfg.pin_d7  = GPIO_NUM_3;   // DB8(G)  -> IO3
      cfg.pin_d8  = GPIO_NUM_46;  // DB9(G)  -> IO46
      cfg.pin_d9  = GPIO_NUM_9;   // DB10(G) -> IO9
      cfg.pin_d10 = GPIO_NUM_10;  // DB11(R) -> IO10
      cfg.pin_d11 = GPIO_NUM_11;  // DB13(R) -> IO11
      cfg.pin_d12 = GPIO_NUM_12;  // DB14(R) -> IO12
      cfg.pin_d13 = GPIO_NUM_13;  // DB15(R) -> IO13
      cfg.pin_d14 = GPIO_NUM_14;  // DB16(R) -> IO14
      cfg.pin_d15 = GPIO_NUM_0;   // DB17(R) -> IO0

      // Control / timing pins
      cfg.pin_henable = GPIO_NUM_18;  // DE    -> IO18
      cfg.pin_vsync   = GPIO_NUM_17;  // VSYNC -> IO17
      cfg.pin_hsync   = GPIO_NUM_16;  // HSYNC -> IO16
      cfg.pin_pclk    = GPIO_NUM_21;  // PCLK  -> IO21

      cfg.freq_write = 16000000; // 16MHz for RGB bus

      // Timing parameters for ST7701 display
      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = 8;
      cfg.hsync_pulse_width = 4;
      cfg.hsync_back_porch  = 8;
      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = 8;
      cfg.vsync_pulse_width = 4;
      cfg.vsync_back_porch  = 8;
      cfg.pclk_idle_high    = 0;
      cfg.de_idle_high      = 0;

      _bus_instance.config(cfg);
    }
    _panel_instance.setBus(&_bus_instance);

    // Backlight (PWM) - DISABLED in LGFX
    // We handle backlight manually because it shares the pin with Touch Reset.
    // Letting LGFX control it would cause touch controller resets.
    /*
    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = TFT_BL_PIN;
      _light_instance.config(cfg);
    }
    _panel_instance.light(&_light_instance);
    */

    // GT911 Touch controller configuration
    // Note: pin_rst is set to -1 so LovyanGFX will not attempt a hardware reset.
    // Several Guition ESP32-4848S040 revisions do not route the GT911 RST/INT pins,
    // so initDisplay() performs a manual reset only when GPIOs are defined.
    {
      auto cfg = _touch_instance.config();
      cfg.x_min = 0;
      cfg.x_max = TFT_WIDTH - 1;
      cfg.y_min = 0;
      cfg.y_max = TFT_HEIGHT - 1;
      cfg.pin_int = TOUCH_INT_PIN;
      cfg.pin_rst = -1;  // Leave GT911 reset unmanaged unless initDisplay() handles it
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;
      cfg.i2c_port = 1;
      cfg.i2c_addr = 0x14;  // Initial probe address; driver falls back to 0x5D if needed
      cfg.pin_sda = TOUCH_SDA_PIN;
      cfg.pin_scl = TOUCH_SCL_PIN;
      cfg.freq = 400000;
      // Touch_GT911::init() automatically retries with 0x5D if 0x14 fails, so no extra logic needed here.
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

// LCD display instance and state variables
static LGFX display;
static bool displayReady = false;
static bool touchReady = false;
static int currentServiceIndex = 0;  // Used for detail view
static bool displayNeedsUpdate = true;
static unsigned long lastTouchTime = 0;
static const unsigned long TOUCH_DEBOUNCE_MS = 300;  // Debounce interval for touch input

// Display view state machine
enum DisplayView {
  VIEW_MAIN,      // Main view showing all services as buttons
  VIEW_DETAIL,    // Detail view showing one service's information
  VIEW_OFF        // Screen is off (backlight off)
};

static DisplayView currentView = VIEW_MAIN;

// Screen timeout and power management
// Screen timeout is configured via SCREEN_TIMEOUT in .env file (in seconds, 0 = disabled)
static unsigned long lastActivityTime = 0;       // Last user interaction time
static unsigned long screenTimeoutMs = 60000;    // Will be set from SCREEN_TIMEOUT config in initDisplay()

// Double-tap detection for wake
static unsigned long lastTapTime = 0;
static const unsigned long DOUBLE_TAP_WINDOW_MS = 500;  // Time window for double-tap

// UI layout constants
static const int HEADER_HEIGHT = 50;
static const int POWER_BUTTON_SIZE = 40;
static const int SERVICE_BUTTON_MARGIN = 10;
static const int SERVICE_BUTTON_HEIGHT = 55;
static const int URL_CHARS_PER_LINE = 45;         // Characters per line when wrapping URLs
static const int URL_MAX_LINES = 3;               // Maximum lines for URL display
static const unsigned long DISPLAY_AUTO_REFRESH_MS = 5000; // Auto-refresh interval for main view

// Function prototypes for LCD
void initDisplay();
void renderMainView();
void renderDetailView();
void handleDisplayLoop();
void handleMainViewTouch(int16_t x, int16_t y);
void handleDetailViewTouch(int16_t x, int16_t y);
void handleScreenOffTouch(int16_t x, int16_t y);
void turnScreenOff();
void turnScreenOn();
void recordActivity();
#endif // HAS_LCD

// RGB LED Status Indicator
// Uses the built-in RGB LED on ESP32-S3 DevKitC (GPIO 48)
// LED states indicate system and service health at a glance

enum LedStatus {
  LED_STATUS_BOOTING,      // Blue pulsing - system booting, no checks yet
  LED_STATUS_NO_WIFI,      // Orange - no WiFi connection
  LED_STATUS_MESHCORE,     // White - communicating with MeshCore radio
  LED_STATUS_ALL_UP,       // Green pulsing - all services are UP
  LED_STATUS_ANY_DOWN,     // Red pulsing - one or more services are DOWN
  LED_STATUS_PAUSED_DOWN   // Orange pulsing - paused services are DOWN
};

// Current LED state
static LedStatus currentLedStatus = LED_STATUS_BOOTING;
static unsigned long lastLedUpdate = 0;
static bool ledPulseDirection = true;  // true = brightening, false = dimming
static uint8_t ledBrightness = 0;

// LED pulsing parameters
const unsigned long LED_PULSE_INTERVAL_MS = 20;  // Update interval for smooth pulsing
const uint8_t LED_PULSE_STEP = 5;               // Brightness change per step
const uint8_t LED_MAX_BRIGHTNESS = 100;         // Max brightness (0-255)
const uint8_t LED_MIN_BRIGHTNESS = 5;           // Min brightness for pulsing

// Update RGB LED based on current status
void updateLed() {
  if (!LED_ENABLED) {
    // If LED is disabled, turn it off
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    return;
  }

  unsigned long now = millis();
  
  // Only update at the pulse interval for smooth animation
  if (now - lastLedUpdate < LED_PULSE_INTERVAL_MS) {
    return;
  }
  lastLedUpdate = now;

  // Calculate pulsing brightness for states that pulse
  bool shouldPulse = (currentLedStatus == LED_STATUS_BOOTING ||
                      currentLedStatus == LED_STATUS_ALL_UP ||
                      currentLedStatus == LED_STATUS_ANY_DOWN ||
                      currentLedStatus == LED_STATUS_PAUSED_DOWN);
  
  if (shouldPulse) {
    if (ledPulseDirection) {
      ledBrightness += LED_PULSE_STEP;
      if (ledBrightness >= LED_MAX_BRIGHTNESS) {
        ledBrightness = LED_MAX_BRIGHTNESS;
        ledPulseDirection = false;
      }
    } else {
      if (ledBrightness > LED_PULSE_STEP + LED_MIN_BRIGHTNESS) {
        ledBrightness -= LED_PULSE_STEP;
      } else {
        ledBrightness = LED_MIN_BRIGHTNESS;
        ledPulseDirection = true;
      }
    }
  } else {
    // Non-pulsing states use steady brightness
    ledBrightness = LED_MAX_BRIGHTNESS;
  }

  // Set color based on status
  uint8_t r = 0, g = 0, b = 0;
  
  switch (currentLedStatus) {
    case LED_STATUS_BOOTING:
      // Blue pulsing
      b = ledBrightness;
      break;
    case LED_STATUS_NO_WIFI:
      // Orange (steady)
      r = ledBrightness;
      g = ledBrightness / 3;  // Orange = red + some green
      break;
    case LED_STATUS_MESHCORE:
      // White (steady)
      r = ledBrightness;
      g = ledBrightness;
      b = ledBrightness;
      break;
    case LED_STATUS_ALL_UP:
      // Green pulsing
      g = ledBrightness;
      break;
    case LED_STATUS_ANY_DOWN:
      // Red pulsing
      r = ledBrightness;
      break;
    case LED_STATUS_PAUSED_DOWN:
      // Orange pulsing - paused services are DOWN
      r = ledBrightness;
      g = ledBrightness / 3;  // Orange = red + some green
      break;
  }

  neopixelWrite(RGB_BUILTIN, r, g, b);
}

// Set LED status and reset pulse state for smooth transition
void setLedStatus(LedStatus status) {
  if (currentLedStatus != status) {
    currentLedStatus = status;
    // Reset pulse for new status
    ledBrightness = LED_MIN_BRIGHTNESS;
    ledPulseDirection = true;
    // Immediate update
    lastLedUpdate = 0;
    updateLed();
  }
}

bool isNtfyConfigured() {
  return strlen(NTFY_TOPIC) > 0;
}

bool isDiscordConfigured() {
  return strlen(DISCORD_WEBHOOK_URL) > 0;
}

bool isSmtpConfigured() {
  return strlen(SMTP_SERVER) > 0 && strlen(SMTP_FROM_ADDRESS) > 0 &&
         strlen(SMTP_TO_ADDRESS) > 0;
}

bool isMeshCoreConfigured() {
#ifdef HAS_LORA_RADIO
  // For LoRa mode, MeshCore is configured if channel name is set
  return strlen(BLE_MESH_CHANNEL_NAME) > 0;
#else
  // For BLE mode, MeshCore is configured if peer name is set AND (channel OR room server is set)
  return strlen(BLE_PEER_NAME) > 0 && 
         (strlen(BLE_MESH_CHANNEL_NAME) > 0 || strlen(BLE_MESH_ROOM_SERVER_ID) > 0);
#endif
}

bool isMeshChannelConfigured() {
  return strlen(BLE_MESH_CHANNEL_NAME) > 0;
}

bool isMeshRoomServerConfigured() {
  return strlen(BLE_MESH_ROOM_SERVER_ID) > 0;
}

void sendNtfyNotification(const String& title, const String& message, const String& tags = "warning,monitor");
void sendDiscordNotification(const String& title, const String& message);
void sendSmtpNotification(const String& title, const String& message);
void sendMeshCoreNotification(const String& title, const String& message);
void initMeshCore();
void disconnectMeshCore();
void disconnectWiFi();
void reconnectWiFi();

AsyncWebServer server(80);

// MeshCore Protocol Stack (layered architecture)
// The transport layer varies based on hardware:
// - BLE mode (default): BLECentralTransport connects to external MeshCore device
// - LoRa mode (HAS_LORA_RADIO): LoRaTransport uses built-in SX1262 radio
#ifdef HAS_LORA_RADIO
static LoRaTransport* meshTransport = nullptr;
static FrameCodec* meshCodec = nullptr;
// In LoRa mode, messages are sent directly - no companion protocol needed
// The LoRa radio is always "connected" once initialized
#else
// Layer 1: BLE Transport - handles BLE connection and I/O
// Layer 2: Frame Codec - handles frame parsing/building  
// Layer 3: Companion Protocol - handles MeshCore protocol logic
static BLECentralTransport* meshTransport = nullptr;
static FrameCodec* meshCodec = nullptr;
static CompanionProtocol* meshProtocol = nullptr;
#endif

// BLE/WiFi coexistence - ESP32-S3 cannot run WiFi and BLE simultaneously
// This is not needed for LoRa mode since LoRa and WiFi can coexist
bool bleOperationInProgress = false;
bool monitoringPaused = false;

// MeshCore protocol constants for LoRa mode
// These match the CompanionProtocol constants used in BLE mode
#ifdef HAS_LORA_RADIO
static constexpr uint8_t LORA_CMD_SEND_CHANNEL_TXT_MSG = 3;
static constexpr uint8_t LORA_TXT_TYPE_PLAIN = 0;
static constexpr uint8_t LORA_DEFAULT_CHANNEL_INDEX = 0;  // Default channel for broadcast
static constexpr size_t LORA_MAX_TEXT_MESSAGE_LEN = 140;

/**
 * Ensure LoRa transport is initialized
 * Creates the transport and codec if not already created
 * @return true if transport is ready to use
 */
bool ensureLoRaTransportInitialized() {
  if (meshTransport != nullptr && meshTransport->isInitialized()) {
    return true;
  }
  
  // Clean up any partial state
  if (meshCodec != nullptr) {
    delete meshCodec;
    meshCodec = nullptr;
  }
  if (meshTransport != nullptr) {
    delete meshTransport;
    meshTransport = nullptr;
  }
  
  // Create and initialize transport
  LoRaTransport::Config config;
  config.pinNss = LORA_NSS;
  config.pinDio1 = LORA_DIO1;
  config.pinRst = LORA_RST;
  config.pinBusy = LORA_BUSY;
  config.pinMosi = LORA_MOSI;
  config.pinMiso = LORA_MISO;
  config.pinSck = LORA_SCK;
  config.frequency = LORA_FREQUENCY;
  config.bandwidth = LORA_BANDWIDTH;
  config.spreadingFactor = LORA_SPREADING_FACTOR;
  config.codingRate = LORA_CODING_RATE;
  config.syncWord = LORA_SYNC_WORD;
  config.txPower = LORA_TX_POWER;
  
  meshTransport = new LoRaTransport(config);
  meshCodec = new FrameCodec(*meshTransport);
  
  if (!meshTransport->init()) {
    Serial.println("ERROR: LoRa radio initialization failed");
    delete meshCodec;
    meshCodec = nullptr;
    delete meshTransport;
    meshTransport = nullptr;
    return false;
  }
  
  return true;
}

/**
 * Build and send a MeshCore channel message via LoRa
 * @param message Message text to send
 * @return true if message was sent successfully
 */
bool sendLoRaChannelMessage(const String& message) {
  if (!ensureLoRaTransportInitialized()) {
    return false;
  }
  
  // Build MeshCore packet for channel message
  std::vector<uint8_t> payload;
  
  payload.push_back(LORA_TXT_TYPE_PLAIN);
  payload.push_back(LORA_DEFAULT_CHANNEL_INDEX);
  
  // Timestamp (little-endian, 4 bytes)
  time_t now = time(nullptr);
  uint32_t timestamp = static_cast<uint32_t>(now);
  payload.push_back(static_cast<uint8_t>(timestamp & 0xFF));
  payload.push_back(static_cast<uint8_t>((timestamp >> 8) & 0xFF));
  payload.push_back(static_cast<uint8_t>((timestamp >> 16) & 0xFF));
  payload.push_back(static_cast<uint8_t>((timestamp >> 24) & 0xFF));
  
  // Message text (truncate if too long)
  size_t textLen = message.length();
  if (textLen > LORA_MAX_TEXT_MESSAGE_LEN) {
    textLen = LORA_MAX_TEXT_MESSAGE_LEN;
  }
  payload.insert(payload.end(), message.begin(), message.begin() + textLen);
  
  // Send using frame codec
  return meshCodec->sendFrame(LORA_CMD_SEND_CHANNEL_TXT_MSG, payload);
}
#endif

// Minimum valid Unix timestamp for NTP validation (2021-01-01 00:00:00 UTC)
// Used to detect if time has been properly synchronized via NTP
const time_t MIN_VALID_TIMESTAMP = 1609459200;

// Pending MeshCore notification - used to defer BLE operations from HTTP handlers and boot
// This prevents task watchdog timeouts by:
// 1. Allowing the async web server to complete HTTP response delivery before WiFi disconnects
// 2. Deferring boot notifications until after setup() completes to avoid watchdog timeouts
volatile bool pendingMeshNotification = false;
String pendingMeshTitle = "";
String pendingMeshMessage = "";

// Helper functions to access protocol state (for API compatibility)
bool isMeshDeviceConnected() {
  return meshTransport != nullptr && meshTransport->isConnected();
}

bool isMeshChannelReady() {
#ifdef HAS_LORA_RADIO
  // For LoRa mode, channel is ready when transport is initialized
  return meshTransport != nullptr && meshTransport->isInitialized();
#else
  return meshProtocol != nullptr && meshProtocol->isChannelReady();
#endif
}

String getMeshLastError() {
#ifdef HAS_LORA_RADIO
  if (meshTransport != nullptr) {
    return meshTransport->getLastError();
  }
  return "";
#else
  if (meshProtocol != nullptr) {
    return meshProtocol->getLastError();
  }
  if (meshTransport != nullptr) {
    return meshTransport->getLastError();
  }
  return "";
#endif
}

int getMeshProtocolState() {
#ifdef HAS_LORA_RADIO
  // For LoRa mode, return 3 (SessionReady equivalent) if initialized
  return (meshTransport != nullptr && meshTransport->isInitialized()) ? 3 : 0;
#else
  if (meshProtocol != nullptr) {
    return static_cast<int>(meshProtocol->getState());
  }
  return 0;
#endif
}

// Service types
// Right now the behavior for each is rudimentary
// However, you can use this to expand and add services with more complex checks
enum ServiceType {
  TYPE_HTTP_GET,
  TYPE_PING,
  TYPE_SNMP_GET,
  TYPE_PORT,
  TYPE_PUSH
};

// SNMP comparison operators for value checks
enum SnmpCompareOp {
  SNMP_OP_EQ,    // Equal (=)
  SNMP_OP_NE,    // Not equal (<>)
  SNMP_OP_LT,    // Less than (<)
  SNMP_OP_LE,    // Less than or equal (<=)
  SNMP_OP_GT,    // Greater than (>)
  SNMP_OP_GE     // Greater than or equal (>=)
};

// Service structure
struct Service {
  String id;
  String name;
  ServiceType type;
  String host;
  int port;
  String path;
  String url;             // Full URL for HTTP GET (http:// or https://)
  String expectedResponse;
  int checkInterval;
  int passThreshold;      // Number of consecutive passes required to mark as UP
  int failThreshold;      // Number of consecutive fails required to mark as DOWN
  int rearmCount;         // Number of failed checks before re-alerting (0 = disabled)
  int consecutivePasses;  // Current count of consecutive passes
  int consecutiveFails;   // Current count of consecutive fails
  int failedChecksSinceAlert; // Failed checks since last alert (for re-arm)
  bool isUp;
  bool hasBeenUp;         // Whether service has ever been UP since boot (for initial UP suppression)
  unsigned long lastCheck;
  unsigned long lastUptime;
  String lastError;
  int secondsSinceLastCheck;
  // SNMP-specific fields
  String snmpOid;         // SNMP OID to query (e.g., "1.3.6.1.2.1.1.1.0")
  String snmpCommunity;   // SNMP community string (default: "public")
  SnmpCompareOp snmpCompareOp;  // Comparison operator for SNMP value check
  String snmpExpectedValue;     // Expected value for comparison
  // Push-specific fields
  String pushToken;       // Unique token for push endpoint (for TYPE_PUSH)
  unsigned long lastPush; // Timestamp of last push received (millis)
  // Enable/disable and pause fields
  bool enabled;           // Whether service checks are enabled
  unsigned long pauseUntil; // Timestamp (millis) until which checks are paused (0 = not paused)
};

// Store up to 20 services
const int MAX_SERVICES = 20;
Service services[MAX_SERVICES];
int serviceCount = 0;

// Filesystem readiness flag to avoid LittleFS access before mount
static bool littleFsReady = false;

// Port check constants
const int PORT_CHECK_TIMEOUT_MS = 5000;  // TCP connection timeout for port checks

// Push check constants
const unsigned long PUSH_TIMING_MARGIN_MS = 5000;  // Margin for push timing checks

// Regex matching constants
const int MAX_REGEX_PATTERN_LENGTH = 256;
const char* REGEX_PREFIX = "regex:";
const int REGEX_PREFIX_LENGTH = 6;

// Pause/enable constants
// Max pause: ~46 days to safely fit within unsigned long (millis() range)
const int MAX_PAUSE_DURATION_SECONDS = 46 * 24 * 60 * 60;  // 46 days
const unsigned long PAUSE_ROLLOVER_THRESHOLD_MS = 7UL * 24UL * 60UL * 60UL * 1000UL;  // 1 week

// Helper function for rollover-safe pause remaining calculation
// Returns pause remaining in milliseconds, or 0 if pause expired/rolled over
inline unsigned long getPauseRemainingMs(unsigned long pauseUntil, unsigned long currentTime) {
  if (pauseUntil == 0) return 0;
  // If currentTime has passed pauseUntil (normal expiry), remaining will wrap to large value
  // If pauseUntil is in the future, remaining will be the actual time left
  // Both cases are handled by the threshold check below
  unsigned long remaining = pauseUntil - currentTime;
  // If remaining is larger than threshold, assume expired or rollover/reboot
  if (remaining > PAUSE_ROLLOVER_THRESHOLD_MS) return 0;
  return remaining;
}

// Notification queue for failed notifications
// Only stores the latest notification per service (isUp state)
// Tracks which notification channels have failed
struct QueuedNotification {
  String serviceId;      // Service ID this notification is for
  String title;          // Notification title
  String message;        // Notification message
  bool isUp;             // true = online notification, false = offline notification
  String tags;           // ntfy tags (for ntfy notifications)
  bool ntfyPending;      // Needs to be sent via ntfy
  bool discordPending;   // Needs to be sent via Discord
  bool smtpPending;      // Needs to be sent via SMTP
  bool meshPending;      // Needs to be sent via MeshCore
  unsigned long lastRetry; // Last retry attempt timestamp
};

const int MAX_QUEUED_NOTIFICATIONS = MAX_SERVICES;
QueuedNotification notificationQueue[MAX_QUEUED_NOTIFICATIONS];
int queuedNotificationCount = 0;

// Retry interval for failed notifications (30 seconds for WiFi-based)
const unsigned long NOTIFICATION_RETRY_INTERVAL = 30000;

// MeshCore retry interval (10 minutes to prevent frequent WiFi disconnects)
const unsigned long MESHCORE_RETRY_INTERVAL = 600000;
unsigned long lastMeshCoreRetry = 0;

// prototype declarations
void initWiFi();
void initWebServer();
void initFileSystem();
bool ensureAuthenticated(AsyncWebServerRequest* request);
void loadServices();
void saveServices();
String generateServiceId();
String generatePushToken();
void checkServices();
void sendOfflineNotification(const Service& service);
void sendOnlineNotification(const Service& service);
void sendSmtpNotification(const String& title, const String& message);
bool checkHttpGet(Service& service);
bool checkPing(Service& service);
bool checkSnmpGet(Service& service);
bool checkPort(Service& service);
bool checkPush(Service& service);
int matchesRegex(const String& text, const String& pattern);
String getWebPage();
String getAdminPage();
String getServiceTypeString(ServiceType type);
String getSnmpCompareOpString(SnmpCompareOp op);
SnmpCompareOp parseSnmpCompareOp(const String& opStr);
String base64Encode(const String& input);
bool readSmtpResponse(WiFiClient& client, int expectedCode);
bool sendSmtpCommand(WiFiClient& client, const String& command, int expectedCode);
void sendBootNotification();

// Notification sending functions that return success status
bool sendNtfyNotificationWithStatus(const String& title, const String& message, const String& tags);
bool sendDiscordNotificationWithStatus(const String& title, const String& message);
bool sendSmtpNotificationWithStatus(const String& title, const String& message);
bool sendMeshCoreNotificationWithStatus(const String& title, const String& message);

// Notification queue functions
void queueNotification(const String& serviceId, const String& title, const String& message, 
                       bool isUp, const String& tags, bool ntfyFailed, bool discordFailed, 
                       bool smtpFailed, bool meshFailed);
void processNotificationQueue();
void processMeshCoreQueue();
int findQueuedNotification(const String& serviceId);
void removeQueuedNotification(int index);

void setup() {
  Serial.begin(115200);
  // Simple delay for serial initialization - matches working ESP32-4848S040 implementation
  // from ESP32-Uptime-Monitoring-Touch. The previous complex !Serial wait loop was causing
  // issues with serial output on this board.
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("   ESP32 Uptime Monitor Starting...");
  Serial.println("========================================");

  // Initialize LED to booting state (blue pulsing)
  setLedStatus(LED_STATUS_BOOTING);

  // Initialize filesystem
  initFileSystem();
  Serial.printf("LittleFS ready: %s\n", littleFsReady ? "yes" : "no");

  // Initialize WiFi
  initWiFi();

  // Load saved services
  loadServices();

  // Initialize web server
  initWebServer();

  // Send boot notification if enabled and WiFi is connected
  if (BOOT_NOTIFICATION_ENABLED) {
    if (WiFi.status() == WL_CONNECTED) {
      sendBootNotification();
    } else {
      Serial.println("Boot notification skipped: WiFi not connected");
    }
  }

#ifdef HAS_LCD
  // Initialize display and touch controller (if connected)
  initDisplay();
#endif

  Serial.println("System ready!");
  Serial.print("Access web interface at: http://");
  Serial.println(WiFi.localIP());
}

void loop() {
  static unsigned long lastCheckTime = 0;
  static bool hasPerformedChecks = false;  // Track if any check has been performed
  unsigned long currentTime = millis();

#ifdef HAS_LCD
  // Re-scan I2C periodically if touch is not ready (helps debug startup issues)
  static unsigned long lastI2CScan = 0;
  if (!touchReady && (currentTime - lastI2CScan > 5000)) {
    lastI2CScan = currentTime;
    Serial.println("Touch not ready, re-scanning I2C...");
    Wire.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN);
    int devicesFound = 0;
    for (byte address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      if (Wire.endTransmission() == 0) {
        Serial.printf("I2C device found at address 0x%02X\n", address);
        devicesFound++;
      }
    }
    if (devicesFound == 0) Serial.println("No I2C devices found");
    Wire.end();
  }
#endif

  // Update LED animation (call frequently for smooth pulsing)
  updateLed();

  // Process pending MeshCore notifications from HTTP handlers
  // This runs in the main loop to avoid blocking the async web server
  // and prevents task watchdog timeouts from WiFi/BLE switching during HTTP responses
  if (pendingMeshNotification && !bleOperationInProgress) {
    // Copy values and clear flag atomically to prevent race conditions with HTTP handler.
    // The HTTP handler checks both pendingMeshNotification and bleOperationInProgress
    // before writing, so setting the flag false first prevents new writes.
    // We use noInterrupts()/interrupts() to ensure the copy is atomic since
    // the async web server runs in a separate FreeRTOS task.
    noInterrupts();
    pendingMeshNotification = false;
    String title = pendingMeshTitle;
    String message = pendingMeshMessage;
    pendingMeshTitle = "";
    pendingMeshMessage = "";
    interrupts();
    
    sendMeshCoreNotification(title, message);
  }

  // Skip service checks if monitoring is paused (e.g., during BLE operations)
  if (monitoringPaused) {
    delay(10);
    return;
  }

  // Update LED status based on WiFi and service state
  // Priority: No WiFi > MeshCore > Service status
  if (WiFi.status() != WL_CONNECTED) {
    setLedStatus(LED_STATUS_NO_WIFI);
  } else if (!hasPerformedChecks && serviceCount > 0) {
    // Still booting - haven't done any checks yet
    setLedStatus(LED_STATUS_BOOTING);
  } else if (serviceCount == 0) {
    // No services configured - show green (nothing to monitor)
    setLedStatus(LED_STATUS_ALL_UP);
  } else {
    // Check if any enabled service is down (either active or paused)
    bool anyActiveDown = false;
    bool anyPausedDown = false;
    unsigned long currentTimeMs = millis();
    for (int i = 0; i < serviceCount; i++) {
      if (services[i].enabled && !services[i].isUp && services[i].lastCheck > 0) {
        // Check if service is paused (rollover-safe)
        if (getPauseRemainingMs(services[i].pauseUntil, currentTimeMs) > 0) {
          anyPausedDown = true;
        } else {
          anyActiveDown = true;
          break;  // Active down takes priority, no need to check further
        }
      }
    }
    // Priority: Active down (red) > Paused down (orange) > All up (green)
    if (anyActiveDown) {
      setLedStatus(LED_STATUS_ANY_DOWN);
    } else if (anyPausedDown) {
      setLedStatus(LED_STATUS_PAUSED_DOWN);
    } else {
      setLedStatus(LED_STATUS_ALL_UP);
    }
  }

  // Check services every 5 seconds
  if (currentTime - lastCheckTime >= 5000) {
    checkServices();
    lastCheckTime = currentTime;
    if (serviceCount > 0) {
      hasPerformedChecks = true;
    }
  }

  // Process notification queue for failed notifications (WiFi-based)
  processNotificationQueue();
  
  // Process MeshCore queue separately (batched, 10 minute interval)
  processMeshCoreQueue();
  
  // Handle OTA update events
  ElegantOTA.loop();

#ifdef HAS_LCD
  // Handle LCD display updates and touch input
  handleDisplayLoop();
#endif

  delay(10);
}

void initWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(1000);
    Serial.print(".");
    attempts++;
    // Update LED during WiFi connection attempts
    updateLed();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Synchronize time via NTP - required for MeshCore message timestamps
    // Uses UTC (gmtOffset=0, daylightOffset=0) for consistent Unix timestamps
    Serial.println("Synchronizing time via NTP...");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    
    // Wait for time to be set (with timeout)
    // Blocking is acceptable here since web server hasn't started yet
    time_t now = 0;
    int ntpAttempts = 0;
    while (now < MIN_VALID_TIMESTAMP && ntpAttempts < 10) {
      delay(500);
      time(&now);
      ntpAttempts++;
    }
    
    if (now >= MIN_VALID_TIMESTAMP) {
      Serial.printf("Time synchronized: %lu\n", (unsigned long)now);
    } else {
      Serial.println("Warning: NTP time sync failed, timestamps may be incorrect");
    }
  } else {
    Serial.println("\nFailed to connect to WiFi!");
    // Set LED to orange to indicate no WiFi
    setLedStatus(LED_STATUS_NO_WIFI);
  }
}

void initMeshCore() {
  Serial.println("Initializing MeshCore protocol stack...");
  Serial.printf("Free heap before init: %d bytes\n", ESP.getFreeHeap());
  
#ifdef HAS_LORA_RADIO
  // LoRa mode: Use built-in SX1262 radio
  LoRaTransport::Config config;
  config.pinNss = LORA_NSS;
  config.pinDio1 = LORA_DIO1;
  config.pinRst = LORA_RST;
  config.pinBusy = LORA_BUSY;
  config.pinMosi = LORA_MOSI;
  config.pinMiso = LORA_MISO;
  config.pinSck = LORA_SCK;
  config.frequency = LORA_FREQUENCY;
  config.bandwidth = LORA_BANDWIDTH;
  config.spreadingFactor = LORA_SPREADING_FACTOR;
  config.codingRate = LORA_CODING_RATE;
  config.syncWord = LORA_SYNC_WORD;
  config.txPower = LORA_TX_POWER;
  
  meshTransport = new LoRaTransport(config);
  meshCodec = new FrameCodec(*meshTransport);
  
  // Initialize the LoRa radio
  if (!meshTransport->init()) {
    Serial.println("ERROR: LoRa radio initialization failed");
    disconnectMeshCore();
    return;
  }
  
  Serial.println("MeshCore LoRa radio ready");
#else
  // BLE mode: Connect to external MeshCore device
  // Create the layered protocol stack
  BLECentralTransport::Config config;
  config.deviceName = BLE_DEVICE_NAME;
  config.peerName = BLE_PEER_NAME;
  config.pairingPin = BLE_PAIRING_PIN;
  
  meshTransport = new BLECentralTransport(config);
  meshCodec = new FrameCodec(*meshTransport);
  meshProtocol = new CompanionProtocol(*meshTransport, *meshCodec);
  
  // Initialize and connect
  if (!meshTransport->init()) {
    Serial.println("ERROR: BLE initialization failed");
    disconnectMeshCore();  // Clean up allocated objects
    return;
  }
  
  if (!meshTransport->connect()) {
    Serial.println("MeshCore connection failed");
    disconnectMeshCore();  // Clean up allocated objects
    return;
  }
  
  // Start session
  if (!meshProtocol->startSession("ESP32-Uptime")) {
    Serial.println("MeshCore session start failed");
    disconnectMeshCore();  // Clean up allocated objects
    return;
  }
  
  // Find the configured channel
  uint8_t channelIdx;
  if (!meshProtocol->findChannelByName(BLE_MESH_CHANNEL_NAME, channelIdx)) {
    Serial.println("Channel not found");
    disconnectMeshCore();  // Clean up allocated objects
    return;
  }
  
  Serial.printf("MeshCore ready on channel %d\n", channelIdx);
#endif
}

bool ensureAuthenticated(AsyncWebServerRequest* request) {
  if (strlen(WEB_AUTH_USERNAME) == 0 || strlen(WEB_AUTH_PASSWORD) == 0) {
    return true;
  }

  if (request->authenticate(WEB_AUTH_USERNAME, WEB_AUTH_PASSWORD)) {
    return true;
  }

  request->requestAuthentication();
  return false;
}

void initFileSystem() {
  littleFsReady = false;

  // First attempt: try to mount without formatting
  if (LittleFS.begin(false)) {
    Serial.println("LittleFS mounted successfully");
    littleFsReady = true;
    return;
  }

  Serial.println("LittleFS mount failed, attempting format...");

  // Second attempt: begin(true) formats automatically on mount failure and
  // handles partition initialization internally, which is more reliable than
  // calling format() directly on corrupted or uninitialized flash.
  if (LittleFS.begin(true)) {
    Serial.println("LittleFS formatted and mounted successfully");
    littleFsReady = true;
    return;
  }

  Serial.println("LittleFS format via begin(true) failed, trying explicit format...");

  // Third attempt: explicit format for edge cases where begin(true) fails
  // This can happen if the partition table is misconfigured
  if (!LittleFS.format()) {
    Serial.println("LittleFS format failed! Check partition table and flash configuration.");
    Serial.println("Ensure a 'littlefs' data partition exists in partitions.csv and matches your flash size.");
    return;
  }

  Serial.println("LittleFS formatted successfully");

  // Now try to mount the freshly formatted filesystem
  if (!LittleFS.begin(false)) {
    Serial.println("Critical: Failed to mount LittleFS after successful format!");
    littleFsReady = false;
    return;
  }

  Serial.println("LittleFS mounted successfully after format");
  littleFsReady = true;
}

void disconnectMeshCore() {
  // Clear callbacks before cleanup to prevent use-after-free.
  // Callbacks could fire during disconnect/deinit and would reference deleted objects.
  if (meshCodec != nullptr) {
    meshCodec->clearCallbacks();
  }
  
#ifdef HAS_LORA_RADIO
  // LoRa mode: Deinitialize the radio
  if (meshCodec != nullptr) {
    delete meshCodec;
    meshCodec = nullptr;
  }
  
  if (meshTransport != nullptr) {
    meshTransport->deinit();
    delete meshTransport;
    meshTransport = nullptr;
  }
  
  Serial.println("MeshCore LoRa radio deinitialized");
#else
  // BLE mode: Clean up the layered protocol stack
  if (meshProtocol != nullptr) {
    delete meshProtocol;
    meshProtocol = nullptr;
  }
  
  if (meshCodec != nullptr) {
    delete meshCodec;
    meshCodec = nullptr;
  }
  
  if (meshTransport != nullptr) {
    meshTransport->disconnect();
    meshTransport->deinit();
    delete meshTransport;
    meshTransport = nullptr;
  }
  
  Serial.println("MeshCore disconnected and deinitialized");
#endif
}

void disconnectWiFi() {
  Serial.println("Disconnecting WiFi for BLE operation...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100); // Allow WiFi to fully shut down
  Serial.println("WiFi disconnected");
}

void reconnectWiFi() {
  Serial.println("Reconnecting WiFi after BLE operation...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi reconnected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to reconnect to WiFi!");
  }
}

void initWebServer() {

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", getWebPage());
  });

  server.on("/admin", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!ensureAuthenticated(request)) {
      return;
    }
    request->send(200, "text/html", getAdminPage());
  });

  server.on("/api/mesh/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["connected"] = isMeshDeviceConnected();
    doc["peerName"] = BLE_PEER_NAME;
    doc["deviceName"] = BLE_DEVICE_NAME;
    doc["channel"] = BLE_MESH_CHANNEL_NAME;
    doc["channelConfigured"] = isMeshChannelConfigured();
    doc["roomServerId"] = BLE_MESH_ROOM_SERVER_ID;
    doc["roomServerConfigured"] = isMeshRoomServerConfigured();
    doc["roomServerPasswordSet"] = strlen(BLE_MESH_ROOM_SERVER_PASSWORD) > 0;
    doc["channelReady"] = isMeshChannelReady();
    doc["protocolState"] = getMeshProtocolState();
    doc["lastError"] = getMeshLastError();
    doc["bleOperationInProgress"] = bleOperationInProgress;
    doc["pendingNotification"] = (bool)pendingMeshNotification;
    doc["monitoringPaused"] = monitoringPaused;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.on("/api/mesh/send", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (!ensureAuthenticated(request)) {
        return;
      }

      // Check if a BLE operation is already in progress or queued
      if (bleOperationInProgress || pendingMeshNotification) {
        request->send(503, "application/json", "{\"error\":\"BLE operation already in progress\"}");
        return;
      }

      JsonDocument doc;
      if (deserializeJson(doc, data, len) != DeserializationError::Ok) {
        request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
      }

      String title = doc["title"] | "Mesh Message";
      String message = doc["message"] | "";
      if (message.length() == 0) {
        request->send(400, "application/json", "{\"error\":\"Missing message\"}");
        return;
      }

      // Queue the notification to be processed in the main loop.
      // This prevents task watchdog timeouts by allowing the async web server
      // to fully complete HTTP response delivery before WiFi is disconnected
      // for BLE operations. The ESP32-S3 cannot run WiFi and BLE simultaneously,
      // so we must ensure the HTTP response is sent before switching to BLE.
      // Use noInterrupts()/interrupts() to ensure atomic write since the main
      // loop() reads these variables from a different FreeRTOS task context.
      noInterrupts();
      pendingMeshTitle = title;
      pendingMeshMessage = message;
      pendingMeshNotification = true;
      interrupts();
      
      request->send(202, "application/json", "{\"success\":true,\"status\":\"queued\"}");
    }
  );

  // get services
  server.on("/api/services", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    JsonArray array = doc["services"].to<JsonArray>();

    unsigned long currentTime = millis();

    for (int i = 0; i < serviceCount; i++) {
      if (services[i].lastCheck > 0) {
        services[i].secondsSinceLastCheck = (currentTime - services[i].lastCheck) / 1000;
      } else {
        services[i].secondsSinceLastCheck = -1; // Never checked
      }

      JsonObject obj = array.add<JsonObject>();
      obj["id"] = services[i].id;
      obj["name"] = services[i].name;
      obj["type"] = getServiceTypeString(services[i].type);
      obj["host"] = services[i].host;
      obj["port"] = services[i].port;
      obj["path"] = services[i].path;
      obj["url"] = services[i].url;
      obj["expectedResponse"] = services[i].expectedResponse;
      obj["checkInterval"] = services[i].checkInterval;
      obj["passThreshold"] = services[i].passThreshold;
      obj["failThreshold"] = services[i].failThreshold;
      obj["rearmCount"] = services[i].rearmCount;
      obj["consecutivePasses"] = services[i].consecutivePasses;
      obj["consecutiveFails"] = services[i].consecutiveFails;
      obj["failedChecksSinceAlert"] = services[i].failedChecksSinceAlert;
      obj["isUp"] = services[i].isUp;
      obj["secondsSinceLastCheck"] = services[i].secondsSinceLastCheck;
      obj["lastError"] = services[i].lastError;
      // SNMP-specific fields
      obj["snmpOid"] = services[i].snmpOid;
      obj["snmpCommunity"] = services[i].snmpCommunity;
      obj["snmpCompareOp"] = getSnmpCompareOpString(services[i].snmpCompareOp);
      obj["snmpExpectedValue"] = services[i].snmpExpectedValue;
      // Push-specific fields
      obj["pushToken"] = services[i].pushToken;
      // Enable/disable and pause fields
      obj["enabled"] = services[i].enabled;
      obj["pauseUntil"] = services[i].pauseUntil;
      // Calculate pause remaining time in seconds (rollover-safe)
      obj["pauseRemaining"] = getPauseRemainingMs(services[i].pauseUntil, currentTime) / 1000;
    }

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // add service
  server.on("/api/services", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (!ensureAuthenticated(request)) {
        return;
      }
      if (serviceCount >= MAX_SERVICES) {
        request->send(400, "application/json", "{\"error\":\"Maximum services reached\"}");
        return;
      }

      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, data, len);

      if (error) {
        request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
      }

      Service newService;
      newService.id = generateServiceId();
      newService.name = doc["name"].as<String>();

      String typeStr = doc["type"].as<String>();
      if (typeStr == "http_get") {
        newService.type = TYPE_HTTP_GET;
      } else if (typeStr == "ping") {
        newService.type = TYPE_PING;
      } else if (typeStr == "snmp_get") {
        newService.type = TYPE_SNMP_GET;
      } else if (typeStr == "port") {
        newService.type = TYPE_PORT;
      } else if (typeStr == "push") {
        newService.type = TYPE_PUSH;
      } else {
        request->send(400, "application/json", "{\"error\":\"Invalid service type\"}");
        return;
      }

      newService.host = doc["host"].as<String>();
      newService.port = doc["port"] | 80;
      newService.path = doc["path"] | "/";
      newService.url = doc["url"].as<String>();
      newService.expectedResponse = doc["expectedResponse"] | "*";
      newService.checkInterval = doc["checkInterval"] | 60;

      int passThreshold = doc["passThreshold"] | 1;
      if (passThreshold < 1) passThreshold = 1;
      newService.passThreshold = passThreshold;

      int failThreshold = doc["failThreshold"] | 3;
      if (failThreshold < 1) failThreshold = 1;
      newService.failThreshold = failThreshold;

      int rearmCount = doc["rearmCount"] | 1440;
      if (rearmCount < 0) rearmCount = 0;
      newService.rearmCount = rearmCount;

      // SNMP-specific fields
      newService.snmpOid = doc["snmpOid"] | "";
      newService.snmpCommunity = doc["snmpCommunity"] | "public";
      String compareOpStr = doc["snmpCompareOp"] | "=";
      newService.snmpCompareOp = parseSnmpCompareOp(compareOpStr);
      newService.snmpExpectedValue = doc["snmpExpectedValue"] | "";

      // Push-specific fields
      if (newService.type == TYPE_PUSH) {
        // Use provided pushToken if available (for editing), otherwise generate new one
        String providedToken = doc["pushToken"] | "";
        if (providedToken.length() > 0) {
          newService.pushToken = providedToken;
        } else {
          newService.pushToken = generatePushToken();
        }
      } else {
        newService.pushToken = "";
      }
      newService.lastPush = 0;

      newService.consecutivePasses = 0;
      newService.consecutiveFails = 0;
      newService.failedChecksSinceAlert = 0;
      newService.isUp = false;
      newService.hasBeenUp = false;
      newService.lastCheck = 0;
      newService.lastUptime = 0;
      newService.lastError = "";
      newService.secondsSinceLastCheck = -1;
      // Enable/disable and pause fields
      newService.enabled = true;
      newService.pauseUntil = 0;

      services[serviceCount++] = newService;
      saveServices();

      JsonDocument response;
      response["success"] = true;
      response["id"] = newService.id;
      if (newService.type == TYPE_PUSH) {
        response["pushToken"] = newService.pushToken;
      }

      String responseStr;
      serializeJson(response, responseStr);
      request->send(200, "application/json", responseStr);
    }
  );

  // delete service
  server.on("/api/services/*", HTTP_DELETE, [](AsyncWebServerRequest *request) {
    if (!ensureAuthenticated(request)) {
      return;
    }
    String path = request->url();
    String serviceId = path.substring(path.lastIndexOf('/') + 1);

    int foundIndex = -1;
    for (int i = 0; i < serviceCount; i++) {
      if (services[i].id == serviceId) {
        foundIndex = i;
        break;
      }
    }

    if (foundIndex == -1) {
      request->send(404, "application/json", "{\"error\":\"Service not found\"}");
      return;
    }

    // Shift services array
    for (int i = foundIndex; i < serviceCount - 1; i++) {
      services[i] = services[i + 1];
    }
    serviceCount--;

    saveServices();
    request->send(200, "application/json", "{\"success\":true}");
  });

  // update service (enable/disable/pause)
  server.on("/api/services/*", HTTP_PATCH, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (!ensureAuthenticated(request)) {
        return;
      }

      // Extract service ID from URL path
      String path = request->url();
      String serviceId = path.substring(path.lastIndexOf('/') + 1);

      // Find the service
      int foundIndex = -1;
      for (int i = 0; i < serviceCount; i++) {
        if (services[i].id == serviceId) {
          foundIndex = i;
          break;
        }
      }

      if (foundIndex == -1) {
        request->send(404, "application/json", "{\"error\":\"Service not found\"}");
        return;
      }

      JsonDocument doc;
      if (deserializeJson(doc, data, len) != DeserializationError::Ok) {
        request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
      }

      // Update enabled status if provided
      if (!doc["enabled"].isNull()) {
        services[foundIndex].enabled = doc["enabled"].as<bool>();
        Serial.printf("Service '%s' enabled set to %s\n", 
                      services[foundIndex].name.c_str(),
                      services[foundIndex].enabled ? "true" : "false");
      }

      // Update pause duration if provided (in seconds, 0 to unpause)
      if (!doc["pauseDuration"].isNull()) {
        int pauseDuration = doc["pauseDuration"].as<int>();
        if (pauseDuration > 0) {
          // Limit pause duration to prevent overflow (MAX_PAUSE_DURATION_SECONDS * 1000 fits in unsigned long)
          if (pauseDuration > MAX_PAUSE_DURATION_SECONDS) {
            pauseDuration = MAX_PAUSE_DURATION_SECONDS;
          }
          // Calculate pause end time - safe because pauseDuration is limited
          unsigned long durationMs = static_cast<unsigned long>(pauseDuration) * 1000UL;
          services[foundIndex].pauseUntil = millis() + durationMs;
          Serial.printf("Service '%s' paused for %d seconds\n", 
                        services[foundIndex].name.c_str(), pauseDuration);
        } else {
          services[foundIndex].pauseUntil = 0;
          Serial.printf("Service '%s' unpaused\n", services[foundIndex].name.c_str());
        }
      }

      saveServices();

      // Build response with current state (rollover-safe)
      unsigned long currentTimeMs = millis();
      JsonDocument response;
      response["success"] = true;
      response["id"] = services[foundIndex].id;
      response["enabled"] = services[foundIndex].enabled;
      response["pauseUntil"] = services[foundIndex].pauseUntil;
      response["pauseRemaining"] = getPauseRemainingMs(services[foundIndex].pauseUntil, currentTimeMs) / 1000;

      String responseStr;
      serializeJson(response, responseStr);
      request->send(200, "application/json", responseStr);
    }
  );

  // export services configuration
  server.on("/api/export", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    JsonArray array = doc["services"].to<JsonArray>();

    for (int i = 0; i < serviceCount; i++) {
      JsonObject obj = array.add<JsonObject>();
      obj["name"] = services[i].name;
      obj["type"] = getServiceTypeString(services[i].type);
      obj["host"] = services[i].host;
      obj["port"] = services[i].port;
      obj["path"] = services[i].path;
      obj["url"] = services[i].url;
      obj["expectedResponse"] = services[i].expectedResponse;
      obj["checkInterval"] = services[i].checkInterval;
      obj["passThreshold"] = services[i].passThreshold;
      obj["failThreshold"] = services[i].failThreshold;
      obj["rearmCount"] = services[i].rearmCount;
      // SNMP-specific fields
      obj["snmpOid"] = services[i].snmpOid;
      obj["snmpCommunity"] = services[i].snmpCommunity;
      obj["snmpCompareOp"] = getSnmpCompareOpString(services[i].snmpCompareOp);
      obj["snmpExpectedValue"] = services[i].snmpExpectedValue;
      // Push-specific fields (token is regenerated on import for security)
      // We don't export the token, just the type
    }

    String response;
    serializeJson(doc, response);

    AsyncWebServerResponse *res = request->beginResponse(200, "application/json", response);
    res->addHeader("Content-Disposition", "attachment; filename=\"monitors-backup.json\"");
    request->send(res);
  });

  // import services configuration
  server.on("/api/import", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (!ensureAuthenticated(request)) {
        return;
      }
      // Limit payload size to 16KB to prevent DoS
      if (total > 16384) {
        request->send(400, "application/json", "{\"error\":\"Payload too large\"}");
        return;
      }

      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, data, len);

      if (error) {
        request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
      }

      JsonArray array = doc["services"];
      if (array.isNull()) {
        request->send(400, "application/json", "{\"error\":\"Missing services array\"}");
        return;
      }

      int importedCount = 0;
      int skippedCount = 0;

      for (JsonObject obj : array) {
        if (serviceCount >= MAX_SERVICES) {
          skippedCount++;
          continue;
        }

        // Validate required fields
        String name = obj["name"].as<String>();
        String host = obj["host"].as<String>();
        String typeStr = obj["type"].as<String>();
        
        // Push type doesn't require host
        if (name.length() == 0) {
          skippedCount++;
          continue;
        }
        
        // Non-push types require host
        if (typeStr != "push" && host.length() == 0) {
          skippedCount++;
          continue;
        }

        ServiceType type;
        if (typeStr == "http_get") {
          type = TYPE_HTTP_GET;
        } else if (typeStr == "ping") {
          type = TYPE_PING;
        } else if (typeStr == "snmp_get") {
          type = TYPE_SNMP_GET;
        } else if (typeStr == "port") {
          type = TYPE_PORT;
        } else if (typeStr == "push") {
          type = TYPE_PUSH;
        } else {
          skippedCount++;
          continue;
        }

        // Validate and constrain numeric values
        int port = obj["port"] | 80;
        if (port < 1 || port > 65535) port = 80;

        int checkInterval = obj["checkInterval"] | 60;
        if (checkInterval < 10) checkInterval = 10;

        int passThreshold = obj["passThreshold"] | 1;
        if (passThreshold < 1) passThreshold = 1;

        int failThreshold = obj["failThreshold"] | 3;
        if (failThreshold < 1) failThreshold = 1;

        int rearmCount = obj["rearmCount"] | 1440;
        if (rearmCount < 0) rearmCount = 0;

        Service newService;
        newService.id = generateServiceId();
        newService.name = name;
        newService.type = type;
        newService.host = host;
        newService.port = port;
        newService.path = obj["path"] | "/";
        newService.url = obj["url"] | "";
        // Backward compatibility: generate URL from host/port/path if URL is empty
        if (newService.url.length() == 0 && type == TYPE_HTTP_GET && host.length() > 0) {
          String protocol = (port == 443) ? "https://" : "http://";
          newService.url = protocol + host + ":" + String(port) + newService.path;
        }
        newService.expectedResponse = obj["expectedResponse"] | "*";
        newService.checkInterval = checkInterval;
        newService.passThreshold = passThreshold;
        newService.failThreshold = failThreshold;
        newService.rearmCount = rearmCount;
        // SNMP-specific fields
        newService.snmpOid = obj["snmpOid"] | "";
        newService.snmpCommunity = obj["snmpCommunity"] | "public";
        String compareOpStr = obj["snmpCompareOp"] | "=";
        newService.snmpCompareOp = parseSnmpCompareOp(compareOpStr);
        newService.snmpExpectedValue = obj["snmpExpectedValue"] | "";
        // Push-specific fields - generate new token on import for security
        if (type == TYPE_PUSH) {
          newService.pushToken = generatePushToken();
        } else {
          newService.pushToken = "";
        }
        newService.lastPush = 0;
        newService.consecutivePasses = 0;
        newService.consecutiveFails = 0;
        newService.failedChecksSinceAlert = 0;
        newService.isUp = false;
        newService.hasBeenUp = false;
        newService.lastCheck = 0;
        newService.lastUptime = 0;
        newService.lastError = "";
        newService.secondsSinceLastCheck = -1;
        // Enable/disable and pause fields - import as enabled and not paused
        newService.enabled = true;
        newService.pauseUntil = 0;

        services[serviceCount++] = newService;
        importedCount++;
      }

      saveServices();

      JsonDocument response;
      response["success"] = true;
      response["imported"] = importedCount;
      response["skipped"] = skippedCount;

      String responseStr;
      serializeJson(response, responseStr);
      request->send(200, "application/json", responseStr);
    }
  );

  // Push endpoint for push-based monitoring
  // Clients send a GET request to /api/push/{token} to register a check pass
  server.on("/api/push/*", HTTP_GET, [](AsyncWebServerRequest *request) {
    String path = request->url();
    String token = path.substring(path.lastIndexOf('/') + 1);
    
    if (token.length() == 0) {
      request->send(400, "application/json", "{\"error\":\"Missing token\"}");
      return;
    }
    
    // Find the service with this push token
    bool found = false;
    for (int i = 0; i < serviceCount; i++) {
      if (services[i].type == TYPE_PUSH && services[i].pushToken == token) {
        unsigned long now = millis();
        services[i].lastPush = now;
        bool wasUp = services[i].isUp;

        // Mark the service as passing immediately
        services[i].lastCheck = now;
        services[i].lastUptime = now;
        services[i].secondsSinceLastCheck = 0;
        services[i].consecutiveFails = 0;
        services[i].failedChecksSinceAlert = 0;
        services[i].lastError = "";

        int requiredPasses = services[i].passThreshold >= 1 ? services[i].passThreshold : 1;
        services[i].consecutivePasses = requiredPasses;
        services[i].isUp = true;

        if (!wasUp) {
          Serial.printf("Push service '%s' marked UP immediately\n", services[i].name.c_str());

#ifdef HAS_LCD
          displayNeedsUpdate = true;
#endif

          if (services[i].hasBeenUp) {
            sendOnlineNotification(services[i]);
          }
          services[i].hasBeenUp = true;
        }
        found = true;
        Serial.printf("Push received for service '%s'\n", services[i].name.c_str());
        
        JsonDocument response;
        response["success"] = true;
        response["service"] = services[i].name;
        response["timestamp"] = services[i].lastPush;
        
        String responseStr;
        serializeJson(response, responseStr);
        request->send(200, "application/json", responseStr);
        return;
      }
    }
    
    if (!found) {
      request->send(404, "application/json", "{\"error\":\"Invalid push token\"}");
    }
  });

#ifdef HAS_LCD
  // Wake screen API endpoint - simulates a screen touch to wake the display
  server.on("/api/screen/wake", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!ensureAuthenticated(request)) {
      return;
    }

    if (!displayReady) {
      request->send(503, "application/json", "{\"error\":\"Display not ready\"}");
      return;
    }

    // Wake the screen as if it was touched
    turnScreenOn();

    request->send(200, "application/json", "{\"success\":true,\"message\":\"Screen woken\"}");
  });
#endif

  // Initialize ElegantOTA for firmware updates via web interface
  // Access the update page at /update
  // Use existing web authentication credentials if configured
  if (strlen(WEB_AUTH_USERNAME) > 0 && strlen(WEB_AUTH_PASSWORD) > 0) {
    ElegantOTA.begin(&server, WEB_AUTH_USERNAME, WEB_AUTH_PASSWORD);
  } else {
    ElegantOTA.begin(&server);
  }
  
  server.begin();
  Serial.println("Web server started");
  Serial.println("OTA update available at: http://<ip>/update");
}

String generateServiceId() {
  return String(millis()) + String(random(1000, 9999));
}

String generatePushToken() {
  // Generate a 16-character hex token for push endpoints
  // Uses ESP32's hardware random number generator for better entropy
  String token = "";
  const char hexChars[] = "0123456789abcdef";
  for (int i = 0; i < 16; i++) {
    token += hexChars[esp_random() % 16];
  }
  return token;
}

void checkServices() {
  // Skip all network-dependent checks if WiFi is not connected
  // This prevents crashes from network operations when WiFi is unavailable
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  unsigned long currentTime = millis();

#ifdef HAS_LCD
  bool anyServiceChecked = false;  // Track if any service was checked this cycle
#endif

  for (int i = 0; i < serviceCount; i++) {
    // Skip disabled services
    if (!services[i].enabled) {
      continue;
    }

    // Skip paused services (using rollover-safe helper function)
    if (services[i].pauseUntil > 0) {
      unsigned long remaining = getPauseRemainingMs(services[i].pauseUntil, currentTime);
      if (remaining > 0) {
        continue;  // Pause is still active
      }
      // Pause has expired or rollover detected - clear it
      services[i].pauseUntil = 0;
    }

    // Check if it's time to check this service
    if (currentTime - services[i].lastCheck < services[i].checkInterval * 1000) {
      continue;
    }

    services[i].lastCheck = currentTime;
    bool wasUp = services[i].isUp;

#ifdef HAS_LCD
    anyServiceChecked = true;  // Mark that at least one service was checked
#endif

    // Perform the actual check
    bool checkResult = false;
    switch (services[i].type) {
      case TYPE_HTTP_GET:
        checkResult = checkHttpGet(services[i]);
        break;
      case TYPE_PING:
        checkResult = checkPing(services[i]);
        break;
      case TYPE_SNMP_GET:
        checkResult = checkSnmpGet(services[i]);
        break;
      case TYPE_PORT:
        checkResult = checkPort(services[i]);
        break;
      case TYPE_PUSH:
        checkResult = checkPush(services[i]);
        break;
    }

    // Update consecutive counters based on check result
    if (checkResult) {
      services[i].consecutivePasses++;
      services[i].consecutiveFails = 0;
      services[i].lastUptime = currentTime;
      services[i].lastError = "";
      services[i].failedChecksSinceAlert = 0;  // Reset re-arm counter on success
    } else {
      services[i].consecutiveFails++;
      services[i].consecutivePasses = 0;
    }

    // Determine new state based on thresholds
    if (!services[i].isUp && services[i].consecutivePasses >= services[i].passThreshold) {
      // Service has passed enough times to be considered UP
      services[i].isUp = true;
      services[i].failedChecksSinceAlert = 0;  // Reset re-arm counter on recovery
    } else if (services[i].isUp && services[i].consecutiveFails >= services[i].failThreshold) {
      // Service has failed enough times to be considered DOWN
      services[i].isUp = false;
    }

    // Log and notify on state changes
    if (wasUp != services[i].isUp) {
      Serial.printf("Service '%s' is now %s (after %d consecutive %s)\n",
        services[i].name.c_str(),
        services[i].isUp ? "UP" : "DOWN",
        services[i].isUp ? services[i].consecutivePasses : services[i].consecutiveFails,
        services[i].isUp ? "passes" : "fails");

#ifdef HAS_LCD
      // Set display update flag BEFORE notifications to ensure display refreshes
      // even if notification operations take a long time or encounter errors
      displayNeedsUpdate = true;
#endif

      if (!services[i].isUp) {
        sendOfflineNotification(services[i]);
        services[i].failedChecksSinceAlert = 0;  // Reset counter after initial alert
      } else if (services[i].hasBeenUp) {
        // Only send online notification if service was previously UP (not initial UP after boot)
        sendOnlineNotification(services[i]);
      }
      
      // Mark that service has been UP at least once (for initial UP notification suppression)
      if (services[i].isUp) {
        services[i].hasBeenUp = true;
      }
    } else if (!services[i].isUp && !checkResult && services[i].rearmCount > 0) {
      // Service is still DOWN and check failed - handle re-arm logic
      services[i].failedChecksSinceAlert++;
      if (services[i].failedChecksSinceAlert >= services[i].rearmCount) {
        Serial.printf("Service '%s' still DOWN - re-arming alert after %d failed checks\n",
          services[i].name.c_str(), services[i].failedChecksSinceAlert);
        sendOfflineNotification(services[i]);
        services[i].failedChecksSinceAlert = 0;  // Reset counter after re-arm alert
      }
    }

#ifdef HAS_LCD
    // Update display if viewing detail view and this specific service was checked
    if (currentView == VIEW_DETAIL && i == currentServiceIndex) {
      displayNeedsUpdate = true;
    }
#endif
  }

#ifdef HAS_LCD
  // Update display if in main view and any service was checked
  // This is done once outside the loop to avoid redundant flag setting
  if (currentView == VIEW_MAIN && anyServiceChecked) {
    displayNeedsUpdate = true;
  }
#endif
}

// Helper function to match text against a POSIX extended regex pattern
// Returns 0 on match, 1 on no match, -1 on pattern too long, -2 on invalid pattern
int matchesRegex(const String& text, const String& pattern) {
  // Limit pattern length to prevent excessive resource usage
  if (pattern.length() > MAX_REGEX_PATTERN_LENGTH) {
    return -1;
  }
  
  regex_t regex;
  int result;
  
  // Compile the regex pattern with extended syntax
  result = regcomp(&regex, pattern.c_str(), REG_EXTENDED | REG_NOSUB);
  if (result != 0) {
    // Pattern compilation failed - do not call regfree() on failed compile
    return -2;
  }
  
  // Execute the regex match
  result = regexec(&regex, text.c_str(), 0, NULL, 0);
  regfree(&regex);
  
  return result == 0 ? 0 : 1;  // 0 = match, 1 = no match
}

bool checkHttpGet(Service& service) {
  HTTPClient http;
  
  // Use the URL field directly - supports both HTTP and HTTPS
  String url = service.url;
  
  // Handle empty URL
  if (url.length() == 0) {
    service.lastError = "URL not configured";
    return false;
  }
  
  // Handle HTTPS URLs by using WiFiClientSecure
  bool isSecure = url.startsWith("https://");
  WiFiClient* client = nullptr;
  WiFiClient plainClient;
  WiFiClientSecure secureClient;

  if (isSecure) {
    secureClient.setInsecure();  // Skip certificate validation for HTTPS targets
    client = &secureClient;
  } else {
    client = &plainClient;
  }

  // Ensure the underlying client stays in scope for the full request lifecycle
  if (!http.begin(*client, url)) {
    service.lastError = "Invalid URL";
    return false;
  }

  http.setTimeout(5000);

  int httpCode = http.GET();
  bool isUp = false;

  if (httpCode > 0) {
    if (httpCode == 200) {
      if (service.expectedResponse == "*") {
        isUp = true;
      } else {
        String payload = http.getString();
        // Check if expectedResponse is a regex pattern (prefixed with "regex:")
        if (service.expectedResponse.startsWith(REGEX_PREFIX)) {
          String pattern = service.expectedResponse.substring(REGEX_PREFIX_LENGTH);
          int regexResult = matchesRegex(payload, pattern);
          if (regexResult == 0) {
            isUp = true;
          } else if (regexResult == -1) {
            service.lastError = "Regex pattern too long";
          } else if (regexResult == -2) {
            service.lastError = "Invalid regex pattern";
          } else {
            service.lastError = "Regex mismatch";
          }
        } else {
          // Plain substring match
          isUp = payload.indexOf(service.expectedResponse) >= 0;
          if (!isUp) {
            service.lastError = "Response mismatch";
          }
        }
      }
    } else {
      service.lastError = "HTTP " + String(httpCode);
    }
  } else {
    service.lastError = "Connection failed: " + String(httpCode);
  }

  http.end();
  return isUp;
}

bool checkPing(Service& service) {
  bool success = Ping.ping(service.host.c_str(), 3);
  if (!success) {
    service.lastError = "Ping timeout";
  }
  return success;
}

bool checkPort(Service& service) {
  WiFiClient client;
  // Attempt TCP connection with configured timeout
  if (client.connect(service.host.c_str(), service.port, PORT_CHECK_TIMEOUT_MS)) {
    client.stop();
    return true;
  }
  service.lastError = "Port closed or unreachable";
  return false;
}

bool checkPush(Service& service) {
  // For push-based services, check if a push was received within the check interval
  unsigned long currentTime = millis();
  
  // If no push has ever been received, treat as not up yet
  if (service.lastPush == 0) {
    service.lastError = "No push received yet";
    return false;
  }
  
  // Check if a push was received within the last check interval period
  // We add some margin to account for timing variations
  unsigned long pushAge = currentTime - service.lastPush;
  unsigned long intervalMs = (unsigned long)service.checkInterval * 1000UL;
  
  if (pushAge <= intervalMs + PUSH_TIMING_MARGIN_MS) {
    return true;
  }
  
  service.lastError = "No push received within interval";
  return false;
}

// Helper function to compare SNMP values based on operator
bool compareSnmpValue(const String& actualValue, SnmpCompareOp op, const String& expectedValue) {
  // Try numeric comparison first
  bool actualIsNumeric = true;
  bool expectedIsNumeric = true;
  float actualNum = 0;
  float expectedNum = 0;
  
  // Check if actual value is numeric
  if (actualValue.length() > 0) {
    char* endPtr;
    actualNum = strtof(actualValue.c_str(), &endPtr);
    if (*endPtr != '\0') {
      actualIsNumeric = false;
    }
  } else {
    actualIsNumeric = false;
  }
  
  // Check if expected value is numeric
  if (expectedValue.length() > 0) {
    char* endPtr;
    expectedNum = strtof(expectedValue.c_str(), &endPtr);
    if (*endPtr != '\0') {
      expectedIsNumeric = false;
    }
  } else {
    expectedIsNumeric = false;
  }
  
  // Use numeric comparison if both are numeric
  if (actualIsNumeric && expectedIsNumeric) {
    switch (op) {
      case SNMP_OP_EQ: return actualNum == expectedNum;
      case SNMP_OP_NE: return actualNum != expectedNum;
      case SNMP_OP_LT: return actualNum < expectedNum;
      case SNMP_OP_LE: return actualNum <= expectedNum;
      case SNMP_OP_GT: return actualNum > expectedNum;
      case SNMP_OP_GE: return actualNum >= expectedNum;
    }
  }
  
  // Fall back to string comparison for non-numeric values
  int cmp = actualValue.compareTo(expectedValue);
  switch (op) {
    case SNMP_OP_EQ: return cmp == 0;
    case SNMP_OP_NE: return cmp != 0;
    case SNMP_OP_LT: return cmp < 0;
    case SNMP_OP_LE: return cmp <= 0;
    case SNMP_OP_GT: return cmp > 0;
    case SNMP_OP_GE: return cmp >= 0;
  }
  
  return false;
}

bool checkSnmpGet(Service& service) {
  // Resolve hostname to IP
  IPAddress targetIP;
  if (!WiFi.hostByName(service.host.c_str(), targetIP)) {
    service.lastError = "DNS resolution failed";
    return false;
  }
  
  // Create SNMP objects for this request
  WiFiUDP udp;
  SNMPManager snmpManager(service.snmpCommunity.c_str());
  SNMPGet snmpRequest(service.snmpCommunity.c_str(), 1);  // SNMP version 2c (0=v1, 1=v2c)
  
  // Initialize SNMP manager
  snmpManager.setUDP(&udp);
  snmpManager.begin();
  
  // Use local variables to avoid thread safety issues
  // Use special marker values to detect if they were updated
  char stringValue[256];
  char* stringValuePtr = stringValue;
  stringValue[0] = '\0';
  
  // Use special marker value that's unlikely to be a real SNMP response
  // INT32_MIN is used as a sentinel to detect if the value was updated
  int intValue = INT32_MIN;
  
  String responseValue = "";
  bool gotResponse = false;
  
  // Add handler for string values (covers most SNMP types including OctetString)
  ValueCallback* stringCallback = snmpManager.addStringHandler(targetIP, service.snmpOid.c_str(), &stringValuePtr);
  
  // Also add integer handler for numeric values
  ValueCallback* intCallback = snmpManager.addIntegerHandler(targetIP, service.snmpOid.c_str(), &intValue);
  
  // Build and send SNMP request - add both callbacks
  snmpRequest.addOIDPointer(stringCallback);
  snmpRequest.addOIDPointer(intCallback);
  snmpRequest.setIP(WiFi.localIP());
  snmpRequest.setUDP(&udp);
  snmpRequest.setRequestID(random(1, 65535));
  
  if (!snmpRequest.sendTo(targetIP)) {
    service.lastError = "Failed to send SNMP request";
    return false;
  }
  snmpRequest.clearOIDList();
  
  // Wait for response with timeout (5 seconds)
  unsigned long startTime = millis();
  const unsigned long timeout = 5000;
  
  while (millis() - startTime < timeout) {
    snmpManager.loop();
    
    // Check if we got a string response (non-empty string)
    if (stringValue[0] != '\0') {
      responseValue = String(stringValue);
      gotResponse = true;
      break;
    }
    
    // Check if we got an integer response (value changed from sentinel)
    if (intValue != INT32_MIN) {
      responseValue = String(intValue);
      gotResponse = true;
      break;
    }
    
    // Yield to prevent watchdog timeout
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  if (!gotResponse) {
    service.lastError = "SNMP timeout";
    return false;
  }
  
  // Compare the received value with expected value
  bool success = compareSnmpValue(responseValue, service.snmpCompareOp, service.snmpExpectedValue);
  
  if (!success) {
    service.lastError = "Value mismatch: got '" + responseValue + "', expected " + 
                        getSnmpCompareOpString(service.snmpCompareOp) + " '" + 
                        service.snmpExpectedValue + "'";
  }
  
  return success;
}

String getSnmpCompareOpString(SnmpCompareOp op) {
  switch (op) {
    case SNMP_OP_EQ: return "=";
    case SNMP_OP_NE: return "<>";
    case SNMP_OP_LT: return "<";
    case SNMP_OP_LE: return "<=";
    case SNMP_OP_GT: return ">";
    case SNMP_OP_GE: return ">=";
    default: return "=";
  }
}

SnmpCompareOp parseSnmpCompareOp(const String& opStr) {
  if (opStr == "=" || opStr == "eq") return SNMP_OP_EQ;
  if (opStr == "<>" || opStr == "ne") return SNMP_OP_NE;
  if (opStr == "<" || opStr == "lt") return SNMP_OP_LT;
  if (opStr == "<=" || opStr == "le") return SNMP_OP_LE;
  if (opStr == ">" || opStr == "gt") return SNMP_OP_GT;
  if (opStr == ">=" || opStr == "ge") return SNMP_OP_GE;
  return SNMP_OP_EQ;  // Default to equal
}

void sendOfflineNotification(const Service& service) {
  if (!isNtfyConfigured() && !isDiscordConfigured() && !isSmtpConfigured() && !isMeshCoreConfigured()) {
    return;
  }

  bool wifiConnected = WiFi.status() == WL_CONNECTED;

  String title = "Service DOWN: " + service.name;
  String message = "Service '" + service.name + "' at " + service.host;
  if (service.port > 0 && service.type != TYPE_PING) {
    message += ":" + String(service.port);
  }
  message += " is offline.";

  if (service.lastError.length() > 0) {
    message += " Error: " + service.lastError;
  }

  String tags = "warning,monitor";
  bool ntfyFailed = false;
  bool discordFailed = false;
  bool smtpFailed = false;
  bool meshFailed = false;

  if (wifiConnected) {
    if (isNtfyConfigured()) {
      if (!sendNtfyNotificationWithStatus(title, message, tags)) {
        ntfyFailed = true;
      }
    }

    if (isDiscordConfigured()) {
      if (!sendDiscordNotificationWithStatus(title, message)) {
        discordFailed = true;
      }
    }

    if (isSmtpConfigured()) {
      if (!sendSmtpNotificationWithStatus(title, message)) {
        smtpFailed = true;
      }
    }
  } else {
    Serial.println("WiFi offline: queueing internet notifications");
    ntfyFailed = isNtfyConfigured();
    discordFailed = isDiscordConfigured();
    smtpFailed = isSmtpConfigured();
  }

  if (isMeshCoreConfigured()) {
    if (!sendMeshCoreNotificationWithStatus(title, message)) {
      meshFailed = true;
    }
  }

  // Queue any failed notifications for retry
  queueNotification(service.id, title, message, false, tags, 
                    ntfyFailed, discordFailed, smtpFailed, meshFailed);
}

void sendOnlineNotification(const Service& service) {
  if (!isNtfyConfigured() && !isDiscordConfigured() && !isSmtpConfigured() && !isMeshCoreConfigured()) {
    return;
  }

  bool wifiConnected = WiFi.status() == WL_CONNECTED;

  String title = "Service UP: " + service.name;
  String message = "Service '" + service.name + "' at " + service.host;
  if (service.port > 0 && service.type != TYPE_PING) {
    message += ":" + String(service.port);
  }
  message += " is back online.";

  String tags = "ok,monitor";
  bool ntfyFailed = false;
  bool discordFailed = false;
  bool smtpFailed = false;
  bool meshFailed = false;

  if (wifiConnected) {
    if (isNtfyConfigured()) {
      if (!sendNtfyNotificationWithStatus(title, message, tags)) {
        ntfyFailed = true;
      }
    }

    if (isDiscordConfigured()) {
      if (!sendDiscordNotificationWithStatus(title, message)) {
        discordFailed = true;
      }
    }

    if (isSmtpConfigured()) {
      if (!sendSmtpNotificationWithStatus(title, message)) {
        smtpFailed = true;
      }
    }
  } else {
    Serial.println("WiFi offline: queueing internet notifications");
    ntfyFailed = isNtfyConfigured();
    discordFailed = isDiscordConfigured();
    smtpFailed = isSmtpConfigured();
  }

  if (isMeshCoreConfigured()) {
    if (!sendMeshCoreNotificationWithStatus(title, message)) {
      meshFailed = true;
    }
  }

  // Queue any failed notifications for retry
  queueNotification(service.id, title, message, true, tags, 
                    ntfyFailed, discordFailed, smtpFailed, meshFailed);
}

void sendNtfyNotification(const String& title, const String& message, const String& tags) {
  HTTPClient http;
  String url = String(NTFY_SERVER) + "/" + NTFY_TOPIC;

  WiFiClientSecure client;
  bool isSecure = url.startsWith("https://");
  if (isSecure) {
    client.setInsecure();
    http.begin(client, url);
  } else {
    http.begin(url);
  }
  http.addHeader("Title", title);
  http.addHeader("Tags", tags);
  http.addHeader("Content-Type", "text/plain");

  if (strlen(NTFY_ACCESS_TOKEN) > 0) {
    http.addHeader("Authorization", "Bearer " + String(NTFY_ACCESS_TOKEN));
  } else if (strlen(NTFY_USERNAME) > 0) {
    http.setAuthorization(NTFY_USERNAME, NTFY_PASSWORD);
  }

  int httpCode = http.POST(message);

  if (httpCode > 0) {
    Serial.printf("ntfy notification sent: %d\n", httpCode);
  } else {
    Serial.printf("Failed to send ntfy notification: %d\n", httpCode);
  }

  http.end();
}

void sendDiscordNotification(const String& title, const String& message) {
  HTTPClient http;
  String url = String(DISCORD_WEBHOOK_URL);

  WiFiClientSecure client;
  bool isSecure = url.startsWith("https://");
  if (isSecure) {
    client.setInsecure();
    http.begin(client, url);
  } else {
    http.begin(url);
  }

  http.addHeader("Content-Type", "application/json");

  JsonDocument doc;
  doc["content"] = "**" + title + "**\n" + message;

  String payload;
  serializeJson(doc, payload);

  int httpCode = http.POST(payload);

  if (httpCode > 0) {
    Serial.printf("Discord notification sent: %d\n", httpCode);
  } else {
    Serial.printf("Failed to send Discord notification: %d\n", httpCode);
  }

  http.end();
}

void sendMeshCoreNotification(const String& title, const String& message) {
#ifdef HAS_LORA_RADIO
  // LoRa mode: Send directly using the built-in radio
  // No WiFi/radio coexistence issues - both can run simultaneously
  
  Serial.println("Starting MeshCore notification (LoRa)...");
  
  // Build the message: combine title and message
  String fullMessage = title + ": " + message;
  
  // Send using helper function
  if (sendLoRaChannelMessage(fullMessage)) {
    Serial.println("MeshCore LoRa notification sent successfully");
  } else {
    if (meshTransport != nullptr) {
      Serial.printf("MeshCore LoRa notification failed: %s\n", meshTransport->getLastError().c_str());
    } else {
      Serial.println("MeshCore LoRa notification failed: transport not initialized");
    }
  }
  
#else
  // BLE mode: ESP32-S3 cannot run WiFi and BLE simultaneously
  // Disconnect WiFi, connect BLE, send message, disconnect BLE, reconnect WiFi
  
  Serial.println("Starting MeshCore notification (BLE operation)...");
  
  // Pause monitoring to prevent false positives during network transition
  monitoringPaused = true;
  bleOperationInProgress = true;
  
  // Disconnect WiFi before starting BLE
  disconnectWiFi();
  
  // Create the layered protocol stack on heap to reduce stack usage.
  // Stack overflow can occur when these large objects are combined with
  // deep BLE callback chains during sendTextMessageToChannel.
  BLECentralTransport::Config config;
  config.deviceName = BLE_DEVICE_NAME;
  config.peerName = BLE_PEER_NAME;
  config.pairingPin = BLE_PAIRING_PIN;
  
  BLECentralTransport* transport = new BLECentralTransport(config);
  FrameCodec* codec = new FrameCodec(*transport);
  CompanionProtocol* protocol = new CompanionProtocol(*transport, *codec);
  
  bool success = false;
  
  // Initialize and connect
  if (transport->init()) {
    if (transport->connect()) {
      // Start session
      if (protocol->startSession("ESP32-Uptime")) {
        // Find the configured channel
        uint8_t channelIdx;
        if (protocol->findChannelByName(BLE_MESH_CHANNEL_NAME, channelIdx)) {
          // Build the message: combine title and message
          String fullMessage = title + ": " + message;
          
          // Send using the protocol layer
          if (protocol->sendTextMessageToChannel(channelIdx, fullMessage)) {
            Serial.println("MeshCore notification sent successfully");
            success = true;
          } else {
            Serial.printf("MeshCore notification failed: send error - %s\n", protocol->getLastError().c_str());
          }
        } else {
          Serial.printf("MeshCore notification skipped: channel not found - %s\n", protocol->getLastError().c_str());
        }
      } else {
        Serial.printf("MeshCore notification skipped: session start failed - %s\n", protocol->getLastError().c_str());
      }
    } else {
      Serial.printf("MeshCore notification skipped: not connected - %s\n", transport->getLastError().c_str());
    }
  } else {
    Serial.printf("MeshCore notification skipped: BLE init failed - %s\n", transport->getLastError().c_str());
  }
  
  // Clear callbacks before cleanup to prevent use-after-free.
  // BLE callbacks could fire during disconnect/deinit and would reference deleted objects.
  codec->clearCallbacks();
  
  // Disconnect BLE and deinitialize to free resources
  transport->disconnect();
  transport->deinit();
  
  // Clean up heap-allocated objects
  delete protocol;
  delete codec;
  delete transport;
  
  // Reconnect WiFi
  reconnectWiFi();
  
  // Resume monitoring
  bleOperationInProgress = false;
  monitoringPaused = false;
  
  Serial.println("MeshCore notification operation complete");
#endif
}

String base64Encode(const String& input) {
  size_t outputLength = 0;
  size_t bufferLength = ((input.length() + 2) / 3) * 4 + 4;
  unsigned char* output = new unsigned char[bufferLength];

  int result = mbedtls_base64_encode(
    output,
    bufferLength,
    &outputLength,
    reinterpret_cast<const unsigned char*>(input.c_str()),
    input.length()
  );

  String encoded = "";
  if (result == 0) {
    encoded = String(reinterpret_cast<char*>(output), outputLength);
  }

  delete[] output;
  return encoded;
}

bool readSmtpResponse(WiFiClient& client, int expectedCode) {
  unsigned long timeout = millis() + 5000;
  String line;
  int code = -1;

  do {
    while (!client.available() && millis() < timeout) {
      delay(10);
    }

    if (!client.available()) {
      Serial.println("SMTP response timeout");
      return false;
    }

    line = client.readStringUntil('\n');
    line.trim();
    if (line.length() >= 3) {
      code = line.substring(0, 3).toInt();
    }
  } while (line.length() >= 4 && line.charAt(3) == '-');

  if (code != expectedCode) {
    Serial.printf("SMTP unexpected response (expected %d): %s\n", expectedCode, line.c_str());
    return false;
  }

  return true;
}

bool sendSmtpCommand(WiFiClient& client, const String& command, int expectedCode) {
  client.println(command);
  return readSmtpResponse(client, expectedCode);
}

void sendSmtpNotification(const String& title, const String& message) {
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  WiFiClient* client = &plainClient;

  if (SMTP_USE_TLS) {
    secureClient.setInsecure();
    client = &secureClient;
  }

  if (!client->connect(SMTP_SERVER, SMTP_PORT)) {
    Serial.println("Failed to connect to SMTP server");
    return;
  }

  if (!readSmtpResponse(*client, 220)) return;
  if (!sendSmtpCommand(*client, "EHLO esp32-monitor", 250)) return;

  if (strlen(SMTP_USERNAME) > 0) {
    if (!sendSmtpCommand(*client, "AUTH LOGIN", 334)) return;
    if (!sendSmtpCommand(*client, base64Encode(SMTP_USERNAME), 334)) return;
    if (!sendSmtpCommand(*client, base64Encode(SMTP_PASSWORD), 235)) return;
  }

  if (!sendSmtpCommand(*client, "MAIL FROM:<" + String(SMTP_FROM_ADDRESS) + ">", 250)) return;

  String recipients = String(SMTP_TO_ADDRESS);
  recipients.replace(" ", "");
  int start = 0;
  while (start < recipients.length()) {
    int commaIndex = recipients.indexOf(',', start);
    if (commaIndex == -1) commaIndex = recipients.length();
    String address = recipients.substring(start, commaIndex);
    if (address.length() > 0) {
      if (!sendSmtpCommand(*client, "RCPT TO:<" + address + ">", 250)) return;
    }
    start = commaIndex + 1;
  }

  if (!sendSmtpCommand(*client, "DATA", 354)) return;

  client->printf("From: <%s>\r\n", SMTP_FROM_ADDRESS);
  client->printf("To: %s\r\n", SMTP_TO_ADDRESS);
  client->printf("Subject: %s\r\n", title.c_str());
  client->println("Content-Type: text/plain; charset=\"UTF-8\"\r\n");
  client->println(message);
  client->println(".");

  if (!readSmtpResponse(*client, 250)) return;
  sendSmtpCommand(*client, "QUIT", 221);
  client->stop();

  Serial.println("SMTP notification sent");
}

void sendBootNotification() {
  if (!isNtfyConfigured() && !isDiscordConfigured() && !isSmtpConfigured() && !isMeshCoreConfigured()) {
    Serial.println("Boot notification: No notification channels configured");
    return;
  }

  Serial.println("Sending boot notification...");

  String title = "ESP32 Uptime Monitor Started";
  String message = "Device has booted and is now monitoring services.";
  message += " IP: " + WiFi.localIP().toString();

  if (isNtfyConfigured()) {
    sendNtfyNotification(title, message, "rocket,monitor");
  }

  if (isDiscordConfigured()) {
    sendDiscordNotification(title, message);
  }

  if (isSmtpConfigured()) {
    sendSmtpNotification(title, message);
  }

  if (isMeshCoreConfigured()) {
#ifdef HAS_LORA_RADIO
    // LoRa mode: Send directly - no WiFi/BLE coexistence issues
    sendMeshCoreNotification(title, message);
#else
    // BLE mode: Queue the notification to be sent after setup() completes
    // This prevents watchdog timeouts during boot by deferring the BLE operation
    // to the main loop where it can be handled safely without blocking setup()
    Serial.println("MeshCore boot notification queued (BLE mode)");
    pendingMeshTitle = title;
    pendingMeshMessage = message;
    pendingMeshNotification = true;
#endif
  }

  Serial.println("Boot notification sent");
}

// Notification functions that return success status for queue management

bool sendNtfyNotificationWithStatus(const String& title, const String& message, const String& tags) {
  HTTPClient http;
  String url = String(NTFY_SERVER) + "/" + NTFY_TOPIC;

  WiFiClientSecure client;
  bool isSecure = url.startsWith("https://");
  if (isSecure) {
    client.setInsecure();
    http.begin(client, url);
  } else {
    http.begin(url);
  }
  http.addHeader("Title", title);
  http.addHeader("Tags", tags);
  http.addHeader("Content-Type", "text/plain");

  if (strlen(NTFY_ACCESS_TOKEN) > 0) {
    http.addHeader("Authorization", "Bearer " + String(NTFY_ACCESS_TOKEN));
  } else if (strlen(NTFY_USERNAME) > 0) {
    http.setAuthorization(NTFY_USERNAME, NTFY_PASSWORD);
  }

  int httpCode = http.POST(message);
  http.end();

  if (httpCode >= 200 && httpCode < 300) {
    Serial.printf("ntfy notification sent: %d\n", httpCode);
    return true;
  } else {
    Serial.printf("Failed to send ntfy notification: %d\n", httpCode);
    return false;
  }
}

bool sendDiscordNotificationWithStatus(const String& title, const String& message) {
  HTTPClient http;
  String url = String(DISCORD_WEBHOOK_URL);

  WiFiClientSecure client;
  bool isSecure = url.startsWith("https://");
  if (isSecure) {
    client.setInsecure();
    http.begin(client, url);
  } else {
    http.begin(url);
  }

  http.addHeader("Content-Type", "application/json");

  JsonDocument doc;
  doc["content"] = "**" + title + "**\n" + message;

  String payload;
  serializeJson(doc, payload);

  int httpCode = http.POST(payload);
  http.end();

  if (httpCode >= 200 && httpCode < 300) {
    Serial.printf("Discord notification sent: %d\n", httpCode);
    return true;
  } else {
    Serial.printf("Failed to send Discord notification: %d\n", httpCode);
    return false;
  }
}

bool sendSmtpNotificationWithStatus(const String& title, const String& message) {
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  WiFiClient* client = &plainClient;

  if (SMTP_USE_TLS) {
    secureClient.setInsecure();
    client = &secureClient;
  }

  if (!client->connect(SMTP_SERVER, SMTP_PORT)) {
    Serial.println("Failed to connect to SMTP server");
    return false;
  }

  if (!readSmtpResponse(*client, 220)) { client->stop(); return false; }
  if (!sendSmtpCommand(*client, "EHLO esp32-monitor", 250)) { client->stop(); return false; }

  if (strlen(SMTP_USERNAME) > 0) {
    if (!sendSmtpCommand(*client, "AUTH LOGIN", 334)) { client->stop(); return false; }
    if (!sendSmtpCommand(*client, base64Encode(SMTP_USERNAME), 334)) { client->stop(); return false; }
    if (!sendSmtpCommand(*client, base64Encode(SMTP_PASSWORD), 235)) { client->stop(); return false; }
  }

  if (!sendSmtpCommand(*client, "MAIL FROM:<" + String(SMTP_FROM_ADDRESS) + ">", 250)) { client->stop(); return false; }

  String recipients = String(SMTP_TO_ADDRESS);
  recipients.replace(" ", "");
  int start = 0;
  while (start < (int)recipients.length()) {
    int commaIndex = recipients.indexOf(',', start);
    if (commaIndex == -1) commaIndex = recipients.length();
    String address = recipients.substring(start, commaIndex);
    if (address.length() > 0) {
      if (!sendSmtpCommand(*client, "RCPT TO:<" + address + ">", 250)) { client->stop(); return false; }
    }
    start = commaIndex + 1;
  }

  if (!sendSmtpCommand(*client, "DATA", 354)) { client->stop(); return false; }

  client->printf("From: <%s>\r\n", SMTP_FROM_ADDRESS);
  client->printf("To: %s\r\n", SMTP_TO_ADDRESS);
  client->printf("Subject: %s\r\n", title.c_str());
  client->println("Content-Type: text/plain; charset=\"UTF-8\"\r\n");
  client->println(message);
  client->println(".");

  if (!readSmtpResponse(*client, 250)) { client->stop(); return false; }
  sendSmtpCommand(*client, "QUIT", 221);
  client->stop();

  Serial.println("SMTP notification sent");
  return true;
}

bool sendMeshCoreNotificationWithStatus(const String& title, const String& message) {
#ifdef HAS_LORA_RADIO
  // LoRa mode: Send directly using the built-in radio
  // No WiFi/radio coexistence issues - both can run simultaneously
  
  Serial.println("Starting MeshCore notification (LoRa)...");
  
  // Set LED to white to indicate MeshCore communication
  setLedStatus(LED_STATUS_MESHCORE);
  
  // Build the message: combine title and message
  String fullMessage = title + ": " + message;
  
  // Send using helper function
  bool success = sendLoRaChannelMessage(fullMessage);
  if (success) {
    Serial.println("MeshCore LoRa notification sent successfully");
  } else {
    if (meshTransport != nullptr) {
      Serial.printf("MeshCore LoRa notification failed: %s\n", meshTransport->getLastError().c_str());
    } else {
      Serial.println("MeshCore LoRa notification failed: transport not initialized");
    }
  }
  
  return success;
  
#else
  // BLE mode: ESP32-S3 cannot run WiFi and BLE simultaneously
  // Disconnect WiFi, connect BLE, send message, disconnect BLE, reconnect WiFi
  
  Serial.println("Starting MeshCore notification (BLE operation)...");
  
  // Set LED to white to indicate MeshCore communication
  setLedStatus(LED_STATUS_MESHCORE);
  
  // Pause monitoring to prevent false positives during network transition
  monitoringPaused = true;
  bleOperationInProgress = true;
  
  // Disconnect WiFi before starting BLE
  disconnectWiFi();
  
  // Create the layered protocol stack on heap to reduce stack usage.
  // Stack overflow can occur when these large objects are combined with
  // deep BLE callback chains during sendTextMessageToChannel.
  BLECentralTransport::Config config;
  config.deviceName = BLE_DEVICE_NAME;
  config.peerName = BLE_PEER_NAME;
  config.pairingPin = BLE_PAIRING_PIN;
  
  BLECentralTransport* transport = new BLECentralTransport(config);
  FrameCodec* codec = new FrameCodec(*transport);
  CompanionProtocol* protocol = new CompanionProtocol(*transport, *codec);
  
  bool success = false;
  bool channelSent = false;
  bool roomServerSent = false;
  
  // Initialize and connect
  if (transport->init()) {
    if (transport->connect()) {
      // Start session
      if (protocol->startSession("ESP32-Uptime")) {
        // Build the message: combine title and message
        String fullMessage = title + ": " + message;
        
        // Send to channel if configured
        if (isMeshChannelConfigured()) {
          uint8_t channelIdx;
          if (protocol->findChannelByName(BLE_MESH_CHANNEL_NAME, channelIdx)) {
            if (protocol->sendTextMessageToChannel(channelIdx, fullMessage)) {
              Serial.println("MeshCore channel notification sent successfully");
              channelSent = true;
            } else {
              Serial.printf("MeshCore channel notification failed: send error - %s\n", protocol->getLastError().c_str());
            }
          } else {
            Serial.printf("MeshCore channel notification skipped: channel not found - %s\n", protocol->getLastError().c_str());
          }
        }
        
        // Send to room server if configured
        if (isMeshRoomServerConfigured()) {
          if (protocol->sendTextMessageToContact(BLE_MESH_ROOM_SERVER_ID, fullMessage, BLE_MESH_ROOM_SERVER_PASSWORD)) {
            Serial.println("MeshCore room server notification sent successfully");
            roomServerSent = true;
          } else {
            Serial.printf("MeshCore room server notification failed: send error - %s\n", protocol->getLastError().c_str());
          }
        }
        
        // Success if at least one destination received the message
        success = channelSent || roomServerSent;
      } else {
        Serial.printf("MeshCore notification skipped: session start failed - %s\n", protocol->getLastError().c_str());
      }
    } else {
      Serial.printf("MeshCore notification skipped: not connected - %s\n", transport->getLastError().c_str());
    }
  } else {
    Serial.printf("MeshCore notification skipped: BLE init failed - %s\n", transport->getLastError().c_str());
  }
  
  // Clear callbacks before cleanup to prevent use-after-free.
  // BLE callbacks could fire during disconnect/deinit and would reference deleted objects.
  codec->clearCallbacks();
  
  // Disconnect BLE and deinitialize to free resources
  transport->disconnect();
  transport->deinit();
  
  // Clean up heap-allocated objects
  delete protocol;
  delete codec;
  delete transport;
  
  // Reconnect WiFi
  reconnectWiFi();
  
  // Resume monitoring
  bleOperationInProgress = false;
  monitoringPaused = false;
  
  Serial.println("MeshCore notification operation complete");
  return success;
#endif
}

// Notification queue helper functions

int findQueuedNotification(const String& serviceId) {
  for (int i = 0; i < queuedNotificationCount; i++) {
    if (notificationQueue[i].serviceId == serviceId) {
      return i;
    }
  }
  return -1;
}

void removeQueuedNotification(int index) {
  if (index < 0 || index >= queuedNotificationCount) return;
  
  // Shift remaining notifications down
  for (int i = index; i < queuedNotificationCount - 1; i++) {
    notificationQueue[i] = notificationQueue[i + 1];
  }
  queuedNotificationCount--;
  Serial.printf("Removed notification from queue, %d remaining\n", queuedNotificationCount);
}

void queueNotification(const String& serviceId, const String& title, const String& message, 
                       bool isUp, const String& tags, bool ntfyFailed, bool discordFailed, 
                       bool smtpFailed, bool meshFailed) {
  // Check if any channel actually failed
  if (!ntfyFailed && !discordFailed && !smtpFailed && !meshFailed) {
    return;  // Nothing to queue
  }
  
  // Look for existing notification for this service
  int existingIndex = findQueuedNotification(serviceId);
  
  if (existingIndex >= 0) {
    // Update existing notification with latest state
    // This ensures we only keep the most recent state (up or down)
    QueuedNotification& existing = notificationQueue[existingIndex];
    existing.title = title;
    existing.message = message;
    existing.isUp = isUp;
    existing.tags = tags;
    existing.ntfyPending = ntfyFailed;
    existing.discordPending = discordFailed;
    existing.smtpPending = smtpFailed;
    existing.meshPending = meshFailed;
    existing.lastRetry = millis();
    Serial.printf("Updated queued notification for service %s (now %s)\n", 
                  serviceId.c_str(), isUp ? "UP" : "DOWN");
  } else {
    // Add new notification to queue
    if (queuedNotificationCount >= MAX_QUEUED_NOTIFICATIONS) {
      Serial.println("Notification queue full, dropping oldest");
      removeQueuedNotification(0);
    }
    
    QueuedNotification& newNotification = notificationQueue[queuedNotificationCount];
    newNotification.serviceId = serviceId;
    newNotification.title = title;
    newNotification.message = message;
    newNotification.isUp = isUp;
    newNotification.tags = tags;
    newNotification.ntfyPending = ntfyFailed;
    newNotification.discordPending = discordFailed;
    newNotification.smtpPending = smtpFailed;
    newNotification.meshPending = meshFailed;
    newNotification.lastRetry = millis();
    queuedNotificationCount++;
    Serial.printf("Queued notification for service %s (%s), %d in queue\n", 
                  serviceId.c_str(), isUp ? "UP" : "DOWN", queuedNotificationCount);
  }
}

void processNotificationQueue() {
  if (queuedNotificationCount == 0) return;
  
  unsigned long currentTime = millis();
  bool wifiConnected = WiFi.status() == WL_CONNECTED;
  
  // Process each queued notification (WiFi-based channels only)
  for (int i = 0; i < queuedNotificationCount; i++) {
    QueuedNotification& notification = notificationQueue[i];
    
    // Check if enough time has passed since last retry
    if (currentTime - notification.lastRetry < NOTIFICATION_RETRY_INTERVAL) {
      continue;
    }
    
    notification.lastRetry = currentTime;
    
    // Retry WiFi-based notifications only if WiFi is connected
    if (wifiConnected) {
      if (notification.ntfyPending && isNtfyConfigured()) {
        if (sendNtfyNotificationWithStatus(notification.title, notification.message, notification.tags)) {
          notification.ntfyPending = false;
          Serial.printf("Retry: ntfy notification sent for %s\n", notification.serviceId.c_str());
        }
      }
      
      if (notification.discordPending && isDiscordConfigured()) {
        if (sendDiscordNotificationWithStatus(notification.title, notification.message)) {
          notification.discordPending = false;
          Serial.printf("Retry: Discord notification sent for %s\n", notification.serviceId.c_str());
        }
      }
      
      if (notification.smtpPending && isSmtpConfigured()) {
        if (sendSmtpNotificationWithStatus(notification.title, notification.message)) {
          notification.smtpPending = false;
          Serial.printf("Retry: SMTP notification sent for %s\n", notification.serviceId.c_str());
        }
      }
    }
    
    // Check if all pending notifications for this service have been sent
    // (MeshCore is processed separately in processMeshCoreQueue)
    if (!notification.ntfyPending && !notification.discordPending && 
        !notification.smtpPending && !notification.meshPending) {
      Serial.printf("All notifications sent for %s, removing from queue\n", notification.serviceId.c_str());
      removeQueuedNotification(i);
      i--;  // Adjust index since we removed an element
    }
  }
}

void processMeshCoreQueue() {
  // Check if MeshCore is configured and if we have any pending MeshCore notifications
  if (!isMeshCoreConfigured() || bleOperationInProgress) return;
  
  // Check if any notifications have pending MeshCore messages
  bool hasPendingMesh = false;
  for (int i = 0; i < queuedNotificationCount; i++) {
    if (notificationQueue[i].meshPending) {
      hasPendingMesh = true;
      break;
    }
  }
  if (!hasPendingMesh) return;
  
  // Check if enough time has passed since last MeshCore retry (10 minutes)
  unsigned long currentTime = millis();
  if (currentTime - lastMeshCoreRetry < MESHCORE_RETRY_INTERVAL) {
    return;
  }
  
#ifdef HAS_LORA_RADIO
  // LoRa mode: Send notifications directly via the built-in radio
  // No WiFi/radio coexistence issues - both can run simultaneously
  
  Serial.println("Processing MeshCore queue (LoRa)...");
  
  // Set LED to white to indicate MeshCore communication
  setLedStatus(LED_STATUS_MESHCORE);
  lastMeshCoreRetry = currentTime;
  
  // Ensure transport is initialized
  if (!ensureLoRaTransportInitialized()) {
    Serial.println("ERROR: LoRa radio initialization failed");
    return;
  }
  
  // Send all pending MeshCore notifications using helper function
  for (int i = 0; i < queuedNotificationCount; i++) {
    QueuedNotification& notification = notificationQueue[i];
    if (notification.meshPending) {
      String fullMessage = notification.title + ": " + notification.message;
      
      if (sendLoRaChannelMessage(fullMessage)) {
        Serial.printf("Retry: MeshCore LoRa notification sent for %s\n", notification.serviceId.c_str());
        notification.meshPending = false;
      } else {
        Serial.printf("MeshCore LoRa send failed for %s: %s\n", 
                      notification.serviceId.c_str(), 
                      meshTransport != nullptr ? meshTransport->getLastError().c_str() : "transport not initialized");
      }
      
      // Small delay between messages
      delay(100);
    }
  }
  
  Serial.println("MeshCore LoRa batch operation complete");
  
#else
  // BLE mode: Need to batch operations due to WiFi/BLE coexistence
  Serial.println("Processing MeshCore queue (batched BLE operation)...");
  
  // Set LED to white to indicate MeshCore communication
  setLedStatus(LED_STATUS_MESHCORE);
  
  // Pause monitoring to prevent false positives during network transition
  monitoringPaused = true;
  bleOperationInProgress = true;
  lastMeshCoreRetry = currentTime;
  
  // Disconnect WiFi before starting BLE
  disconnectWiFi();
  
  // Create the layered protocol stack on heap to reduce stack usage.
  // Stack overflow can occur when these large objects are combined with
  // deep BLE callback chains during sendTextMessageToChannel.
  BLECentralTransport::Config config;
  config.deviceName = BLE_DEVICE_NAME;
  config.peerName = BLE_PEER_NAME;
  config.pairingPin = BLE_PAIRING_PIN;
  
  BLECentralTransport* transport = new BLECentralTransport(config);
  FrameCodec* codec = new FrameCodec(*transport);
  CompanionProtocol* protocol = new CompanionProtocol(*transport, *codec);
  
  bool sessionReady = false;
  uint8_t channelIdx = 0;
  bool channelFound = false;
  
  // Initialize, connect and prepare session once
  // Note: sessionReady indicates the MeshCore protocol session is established,
  // independent of whether a channel is found. Messages can be sent to room server
  // even if channel lookup fails.
  if (transport->init()) {
    if (transport->connect()) {
      if (protocol->startSession("ESP32-Uptime")) {
        sessionReady = true;
        Serial.println("MeshCore session ready");
        
        // Find channel if configured
        if (isMeshChannelConfigured()) {
          if (protocol->findChannelByName(BLE_MESH_CHANNEL_NAME, channelIdx)) {
            channelFound = true;
            Serial.printf("MeshCore batch: channel found at index %d\n", channelIdx);
          } else {
            Serial.printf("MeshCore batch: channel not found - %s\n", protocol->getLastError().c_str());
          }
        }
      } else {
        Serial.printf("MeshCore batch: session start failed - %s\n", protocol->getLastError().c_str());
      }
    } else {
      Serial.printf("MeshCore batch: not connected - %s\n", transport->getLastError().c_str());
    }
  } else {
    Serial.printf("MeshCore batch: BLE init failed - %s\n", transport->getLastError().c_str());
  }
  
  // Send all pending MeshCore notifications in this single session
  if (sessionReady) {
    for (int i = 0; i < queuedNotificationCount; i++) {
      QueuedNotification& notification = notificationQueue[i];
      if (notification.meshPending) {
        String fullMessage = notification.title + ": " + notification.message;
        bool sent = false;
        
        // Send to channel if configured and found
        if (channelFound) {
          if (protocol->sendTextMessageToChannel(channelIdx, fullMessage)) {
            Serial.printf("Retry: MeshCore channel notification sent for %s\n", notification.serviceId.c_str());
            sent = true;
          } else {
            Serial.printf("MeshCore channel send failed for %s: %s\n", 
                          notification.serviceId.c_str(), protocol->getLastError().c_str());
          }
        }
        
        // Send to room server if configured
        if (isMeshRoomServerConfigured()) {
          if (protocol->sendTextMessageToContact(BLE_MESH_ROOM_SERVER_ID, fullMessage, BLE_MESH_ROOM_SERVER_PASSWORD)) {
            Serial.printf("Retry: MeshCore room server notification sent for %s\n", notification.serviceId.c_str());
            sent = true;
          } else {
            Serial.printf("MeshCore room server send failed for %s: %s\n", 
                          notification.serviceId.c_str(), protocol->getLastError().c_str());
          }
        }
        
        if (sent) {
          notification.meshPending = false;
        }
        
        // Small delay between messages to avoid overwhelming the receiver
        delay(100);
      }
    }
  }
  
  // Clear callbacks before cleanup to prevent use-after-free.
  // BLE callbacks could fire during disconnect/deinit and would reference deleted objects.
  codec->clearCallbacks();
  
  // Disconnect BLE and deinitialize to free resources
  transport->disconnect();
  transport->deinit();
  
  // Clean up heap-allocated objects
  delete protocol;
  delete codec;
  delete transport;
  
  // Reconnect WiFi
  reconnectWiFi();
  
  // Resume monitoring
  bleOperationInProgress = false;
  monitoringPaused = false;
  
  Serial.println("MeshCore batch operation complete");
#endif
  
  // Clean up any notifications that have all channels sent
  for (int i = 0; i < queuedNotificationCount; i++) {
    QueuedNotification& notification = notificationQueue[i];
    if (!notification.ntfyPending && !notification.discordPending && 
        !notification.smtpPending && !notification.meshPending) {
      Serial.printf("All notifications sent for %s, removing from queue\n", notification.serviceId.c_str());
      removeQueuedNotification(i);
      i--;
    }
  }
}

void saveServices() {
  if (!littleFsReady) {
    Serial.println("LittleFS not mounted; skipping saveServices");
    return;
  }

  File file = LittleFS.open("/services.json", "w");
  if (!file) {
    Serial.println("Failed to open services.json for writing");
    return;
  }

  JsonDocument doc;
  JsonArray array = doc["services"].to<JsonArray>();

  for (int i = 0; i < serviceCount; i++) {
    JsonObject obj = array.add<JsonObject>();
    obj["id"] = services[i].id;
    obj["name"] = services[i].name;
    obj["type"] = (int)services[i].type;
    obj["host"] = services[i].host;
    obj["port"] = services[i].port;
    obj["path"] = services[i].path;
    obj["url"] = services[i].url;
    obj["expectedResponse"] = services[i].expectedResponse;
    obj["checkInterval"] = services[i].checkInterval;
    obj["passThreshold"] = services[i].passThreshold;
    obj["failThreshold"] = services[i].failThreshold;
    obj["rearmCount"] = services[i].rearmCount;
    // SNMP-specific fields
    obj["snmpOid"] = services[i].snmpOid;
    obj["snmpCommunity"] = services[i].snmpCommunity;
    obj["snmpCompareOp"] = (int)services[i].snmpCompareOp;
    obj["snmpExpectedValue"] = services[i].snmpExpectedValue;
    // Push-specific fields
    obj["pushToken"] = services[i].pushToken;
    // Enable/disable and pause fields
    obj["enabled"] = services[i].enabled;
    obj["pauseUntil"] = services[i].pauseUntil;
  }

  if (serializeJson(doc, file) == 0) {
    Serial.println("Failed to serialize services.json");
  }
  file.close();
  Serial.println("Services saved");
}

void loadServices() {
  if (!littleFsReady) {
    Serial.println("LittleFS not mounted; skipping loadServices");
    return;
  }

  File file = LittleFS.open("/services.json", "r");
  if (!file) {
    Serial.println("No services.json found, starting fresh");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println("Failed to parse services.json");
    return;
  }

  JsonArray array = doc["services"];
  serviceCount = 0;

  for (JsonObject obj : array) {
    if (serviceCount >= MAX_SERVICES) break;

    services[serviceCount].id = obj["id"].as<String>();
    services[serviceCount].name = obj["name"].as<String>();
    services[serviceCount].type = (ServiceType)obj["type"].as<int>();
    services[serviceCount].host = obj["host"].as<String>();
    services[serviceCount].port = obj["port"];
    services[serviceCount].path = obj["path"].as<String>();
    services[serviceCount].url = obj["url"] | "";
    // Backward compatibility: generate URL from host/port/path if URL is empty
    if (services[serviceCount].url.length() == 0 && 
        services[serviceCount].type == TYPE_HTTP_GET &&
        services[serviceCount].host.length() > 0) {
      // Use HTTPS if port is 443, otherwise use HTTP
      String protocol = (services[serviceCount].port == 443) ? "https://" : "http://";
      services[serviceCount].url = protocol + services[serviceCount].host + 
                                   ":" + String(services[serviceCount].port) + 
                                   services[serviceCount].path;
    }
    services[serviceCount].expectedResponse = obj["expectedResponse"].as<String>();
    services[serviceCount].checkInterval = obj["checkInterval"];
    services[serviceCount].passThreshold = obj["passThreshold"] | 1;
    services[serviceCount].failThreshold = obj["failThreshold"] | 3;
    services[serviceCount].rearmCount = obj["rearmCount"] | 1440;
    // SNMP-specific fields
    services[serviceCount].snmpOid = obj["snmpOid"] | "";
    services[serviceCount].snmpCommunity = obj["snmpCommunity"] | "public";
    services[serviceCount].snmpCompareOp = (SnmpCompareOp)(obj["snmpCompareOp"].as<int>());
    services[serviceCount].snmpExpectedValue = obj["snmpExpectedValue"] | "";
    // Push-specific fields
    services[serviceCount].pushToken = obj["pushToken"] | "";
    services[serviceCount].lastPush = 0;
    services[serviceCount].consecutivePasses = 0;
    services[serviceCount].consecutiveFails = 0;
    services[serviceCount].failedChecksSinceAlert = 0;
    services[serviceCount].isUp = false;
    services[serviceCount].hasBeenUp = false;
    services[serviceCount].lastCheck = 0;
    services[serviceCount].lastUptime = 0;
    services[serviceCount].lastError = "";
    services[serviceCount].secondsSinceLastCheck = -1;
    // Enable/disable and pause fields
    services[serviceCount].enabled = obj["enabled"] | true;  // Default to enabled
    services[serviceCount].pauseUntil = obj["pauseUntil"] | 0;

    serviceCount++;
  }

  Serial.printf("Loaded %d services\n", serviceCount);
}

String getServiceTypeString(ServiceType type) {
  switch (type) {
    case TYPE_HTTP_GET: return "http_get";
    case TYPE_PING: return "ping";
    case TYPE_SNMP_GET: return "snmp_get";
    case TYPE_PORT: return "port";
    case TYPE_PUSH: return "push";
    default: return "unknown";
  }
}

// --- LCD and Touch Screen Functions (Conditional) ---
#ifdef HAS_LCD

// Record user activity for screen timeout
void recordActivity() {
  lastActivityTime = millis();
}

// Turn screen off (backlight off)
void turnScreenOff() {
  if (!displayReady) return;
  // Cannot turn off backlight (GPIO 38) because it resets touch.
  // Cannot use PWM (dimming) because it resets touch.
  // Solution: Keep backlight ON (HIGH) and draw black screen to simulate off.
  if (TFT_BL_PIN >= 0) {
    digitalWrite(TFT_BL_PIN, HIGH);
  }
  display.fillScreen(TFT_BLACK);
  currentView = VIEW_OFF;
  Serial.println("Screen 'turned off' (Black screen, BL On)");
}

// Turn screen on (restore backlight)
void turnScreenOn() {
  if (!displayReady) return;
  // Ensure backlight is ON (HIGH)
  if (TFT_BL_PIN >= 0) {
    digitalWrite(TFT_BL_PIN, HIGH);
  }
  currentView = VIEW_MAIN;
  recordActivity();
  displayNeedsUpdate = true;
  Serial.println("Screen turned on");
}

void initDisplay() {
  Serial.println("Initializing display...");
  
  // Reset the GT911 touch controller with proper I2C address selection.
  // The GT911 I2C address is determined by the state of the INT pin during the
  // rising edge of RST:
  //   - INT HIGH during RST rise -> address 0x14
  //   - INT LOW during RST rise -> address 0x5D
  // Hardware note: On ESP32-4848S040, GPIO 38 is shared between touch reset and backlight.
  // After reset, LovyanGFX reconfigures this pin for PWM backlight control.
  if (TOUCH_RST_PIN >= 0) {
    // Configure INT pin as output and set HIGH to select address 0x14
    if (TOUCH_INT_PIN >= 0) {
      pinMode(TOUCH_INT_PIN, OUTPUT);
      digitalWrite(TOUCH_INT_PIN, HIGH);
      delay(5);  // Ensure INT state is stable before reset sequence
    }

    // Perform reset sequence - timing is critical for GT911 address selection
    pinMode(TOUCH_RST_PIN, OUTPUT);
    digitalWrite(TOUCH_RST_PIN, LOW);
    delay(20);  // GT911 requires minimum 10ms reset pulse, use 20ms for margin
    digitalWrite(TOUCH_RST_PIN, HIGH);
    delay(100);  // Wait for GT911 address latch and internal initialization

    // Release INT pin for normal interrupt operation
    if (TOUCH_INT_PIN >= 0) {
      pinMode(TOUCH_INT_PIN, INPUT);
    }

    // Additional delay to allow GT911 I2C to stabilize after INT released
    delay(50);

    Serial.println("GT911 touch controller reset complete");
  } else {
    Serial.println("GT911 reset pin not managed (TOUCH_RST_PIN < 0); skipping manual reset");
  }

  // Debug: Scan I2C bus to verify GT911 presence and address
  Serial.printf("Scanning I2C bus (SDA=%d, SCL=%d)...\n", TOUCH_SDA_PIN, TOUCH_SCL_PIN);

  // Initialize Wire with explicit pins
  if (!Wire.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN)) {
    Serial.println("Failed to initialize I2C bus!");
  } else {
    int devicesFound = 0;
    uint8_t foundAddr = 0;
    for (byte address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      byte error = Wire.endTransmission();
      if (error == 0) {
        Serial.printf("I2C device found at address 0x%02X\n", address);
        devicesFound++;
        if (address == 0x14 || address == 0x5D) {
          foundAddr = address;
        }
      }
    }
    if (devicesFound == 0) Serial.println("No I2C devices found");

    if (foundAddr == 0x14) {
      Serial.println("GT911 detected at 0x14 (INT held high during boot)");
    } else if (foundAddr == 0x5D) {
      Serial.println("GT911 detected at 0x5D (default address)");
    }

    // Important: End Wire so LGFX can initialize its own I2C driver
    Wire.end();
    delay(50);  // Ensure bus is free
  }
  
  displayReady = display.init();
  display.setRotation(0);  // Rotation 0 for ESP32-4848S040 square display
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);

  if (TFT_BL_PIN >= 0) {
    // Manually enable backlight (GPIO 38 HIGH)
    // This also takes the Touch Controller out of Reset
    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, HIGH);
    // Give GT911 time to boot after taking it out of reset
    delay(200);
  }

  // Verify touch controller is working by checking if touch object exists and
  // attempting a test read. The touch object existing just means it was configured,
  // not that it successfully initialized and can communicate with the hardware.
  touchReady = false;
  lgfx::ITouch* touchController = display.touch();
  if (touchController != nullptr) {
    // Get the touch configuration for debug logging
    auto touchCfg = touchController->config();
    Serial.printf("Touch controller configured: I2C addr=0x%02X, INT pin=%d\n", 
                  touchCfg.i2c_addr, touchCfg.pin_int);
    
    // Attempt multiple test touch reads to verify I2C communication is working.
    // The GT911 may need a few polls to fully wake up and respond reliably.
    lgfx::touch_point_t tp;
    
    for (int attempt = 0; attempt < 5; attempt++) {
      // getTouch returns the number of touch points detected (0 or more)
      // Even if no touch is active, a successful read returns 0 (not -1 or error)
      // The important thing is that the call doesn't crash and the internal
      // state machine progresses
      display.getTouch(&tp, 1);
      delay(20);
    }
    
    // After triggering the initialization through reads, we consider it ready
    // if the touch object is still valid. The GT911 driver in LovyanGFX will
    // try both I2C addresses and set _inited if successful.
    touchReady = true;
    Serial.println("Touch controller (GT911) initialized successfully");
  } else {
    Serial.println("Touch controller not configured or not detected");
  }

  if (displayReady) {
    // Initialize screen timeout from config (SCREEN_TIMEOUT in seconds, 0 = disabled)
    // Validate: must be 0 (disabled) or between 10-600 seconds
    if (SCREEN_TIMEOUT == 0) {
      screenTimeoutMs = 0;  // Disabled
      Serial.println("Screen timeout disabled via config");
    } else if (SCREEN_TIMEOUT >= 10 && SCREEN_TIMEOUT <= 600) {
      screenTimeoutMs = (unsigned long)SCREEN_TIMEOUT * 1000;
      Serial.printf("Screen timeout set from config: %d seconds\n", SCREEN_TIMEOUT);
    } else {
      // Invalid value, use default 60 seconds
      screenTimeoutMs = 60000;
      Serial.printf("Invalid SCREEN_TIMEOUT value (%d), using default 60 seconds\n", SCREEN_TIMEOUT);
    }
    
    display.startWrite();
    display.fillScreen(TFT_BLACK);
    display.endWrite();
    currentView = VIEW_MAIN;
    recordActivity();
    displayNeedsUpdate = true;
    Serial.println("Display initialized successfully");
  } else {
    Serial.println("Display initialization failed");
  }
}

// Draw the header with title and power button
void drawHeader() {
  int16_t width = display.width();
  
  // Header background
  display.fillRect(0, 0, width, HEADER_HEIGHT, TFT_NAVY);
  
  // Title
  display.setTextColor(TFT_CYAN, TFT_NAVY);
  display.setTextSize(2);
  display.setCursor(10, 15);
  display.print("ESP32 Monitor");
  
  // WiFi status
  if (WiFi.status() == WL_CONNECTED) {
    display.setTextColor(TFT_WHITE, TFT_NAVY);
    display.setCursor(10, 32);
    display.setTextSize(1);
    display.print(WiFi.localIP().toString());
  }
  
  // Power button (top right)
  int powerBtnX = width - POWER_BUTTON_SIZE - 5;
  int powerBtnY = 5;
  display.fillRoundRect(powerBtnX, powerBtnY, POWER_BUTTON_SIZE, POWER_BUTTON_SIZE, 8, TFT_DARKGREY);
  display.drawRoundRect(powerBtnX, powerBtnY, POWER_BUTTON_SIZE, POWER_BUTTON_SIZE, 8, TFT_WHITE);
  
  // Power icon (simple circle with line)
  int iconCenterX = powerBtnX + POWER_BUTTON_SIZE / 2;
  int iconCenterY = powerBtnY + POWER_BUTTON_SIZE / 2;
  display.drawCircle(iconCenterX, iconCenterY, 10, TFT_WHITE);
  display.fillRect(iconCenterX - 2, iconCenterY - 12, 4, 10, TFT_WHITE);
}

// Get status color for a service
uint16_t getServiceStatusColor(const Service& svc) {
  if (!svc.enabled) {
    return TFT_DARKGREY;
  }
  if (svc.pauseUntil > 0 && getPauseRemainingMs(svc.pauseUntil, millis()) > 0) {
    return TFT_ORANGE;
  }
  if (svc.lastCheck == 0) {
    return TFT_BLUE;  // Pending
  }
  return svc.isUp ? TFT_GREEN : TFT_RED;
}

// Render main view showing all services as buttons
void renderMainView() {
  if (!displayReady) return;

  display.startWrite();
  display.fillScreen(TFT_BLACK);
  
  int16_t width = display.width();
  int16_t height = display.height();

  // Draw header
  drawHeader();

  // Show message if no services configured
  if (serviceCount == 0) {
    display.setCursor(10, HEADER_HEIGHT + 20);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextSize(2);
    display.println("No services configured.");
    display.setCursor(10, HEADER_HEIGHT + 50);
    display.println("Add services via web UI.");
    display.endWrite();
    return;
  }

  // Calculate button layout - 2 columns
  int cols = 2;
  int buttonWidth = (width - (cols + 1) * SERVICE_BUTTON_MARGIN) / cols;
  int startY = HEADER_HEIGHT + SERVICE_BUTTON_MARGIN;
  int availableHeight = height - startY - SERVICE_BUTTON_MARGIN;
  int maxRows = availableHeight / (SERVICE_BUTTON_HEIGHT + SERVICE_BUTTON_MARGIN);
  
  // Draw service buttons
  for (int i = 0; i < serviceCount && i < maxRows * cols; i++) {
    int row = i / cols;
    int col = i % cols;
    
    int btnX = SERVICE_BUTTON_MARGIN + col * (buttonWidth + SERVICE_BUTTON_MARGIN);
    int btnY = startY + row * (SERVICE_BUTTON_HEIGHT + SERVICE_BUTTON_MARGIN);
    
    Service& svc = services[i];
    uint16_t statusColor = getServiceStatusColor(svc);
    
    // Button background
    display.fillRoundRect(btnX, btnY, buttonWidth, SERVICE_BUTTON_HEIGHT, 8, TFT_DARKGREY);
    
    // Status indicator (left side colored bar)
    display.fillRoundRect(btnX, btnY, 8, SERVICE_BUTTON_HEIGHT, 4, statusColor);
    
    // Service name (truncate if too long)
    display.setTextColor(TFT_WHITE, TFT_DARKGREY);
    display.setTextSize(2);
    String name = svc.name;
    if (name.length() > 12) {
      name = name.substring(0, 10) + "..";
    }
    display.setCursor(btnX + 15, btnY + 10);
    display.print(name);
    
    // Status text
    display.setTextSize(1);
    display.setTextColor(statusColor, TFT_DARKGREY);
    display.setCursor(btnX + 15, btnY + 35);
    if (!svc.enabled) {
      display.print("DISABLED");
    } else if (svc.pauseUntil > 0 && getPauseRemainingMs(svc.pauseUntil, millis()) > 0) {
      display.print("PAUSED");
    } else if (svc.lastCheck == 0) {
      display.print("PENDING");
    } else {
      display.print(svc.isUp ? "UP" : "DOWN");
    }
  }
  
  // Show indicator if there are more services than can fit
  if (serviceCount > maxRows * cols) {
    display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    display.setTextSize(1);
    display.setCursor(width / 2 - 40, height - 15);
    display.printf("+ %d more", serviceCount - maxRows * cols);
  }
  display.endWrite();
}

// Render detail view for a single service
void renderDetailView() {
  if (!displayReady) return;
  if (currentServiceIndex >= serviceCount) {
    currentView = VIEW_MAIN;
    displayNeedsUpdate = true;
    return;
  }

  display.startWrite();
  display.fillScreen(TFT_BLACK);
  
  int16_t width = display.width();
  int16_t height = display.height();

  Service& svc = services[currentServiceIndex];
  
  // Header with back button
  display.fillRect(0, 0, width, HEADER_HEIGHT, TFT_NAVY);
  
  // Back button
  display.fillRoundRect(5, 5, 60, POWER_BUTTON_SIZE, 8, TFT_DARKGREY);
  display.drawRoundRect(5, 5, 60, POWER_BUTTON_SIZE, 8, TFT_WHITE);
  display.setTextColor(TFT_WHITE, TFT_DARKGREY);
  display.setTextSize(2);
  display.setCursor(15, 15);
  display.print("<-");
  
  // Service name in header
  display.setTextColor(TFT_CYAN, TFT_NAVY);
  display.setCursor(75, 15);
  String headerName = svc.name;
  if (headerName.length() > 18) {
    headerName = headerName.substring(0, 16) + "..";
  }
  display.print(headerName);

  // Content area
  int contentY = HEADER_HEIGHT + 10;
  
  // Status box with large indicator
  uint16_t statusColor = getServiceStatusColor(svc);
  String statusText;
  if (!svc.enabled) {
    statusText = "DISABLED";
  } else if (svc.pauseUntil > 0 && getPauseRemainingMs(svc.pauseUntil, millis()) > 0) {
    statusText = "PAUSED";
  } else if (svc.lastCheck == 0) {
    statusText = "PENDING";
  } else {
    statusText = svc.isUp ? "UP" : "DOWN";
  }
  
  display.fillRoundRect(10, contentY, width - 20, 50, 10, statusColor);
  display.setTextColor(TFT_WHITE, statusColor);
  display.setTextSize(3);
  display.setCursor(width / 2 - statusText.length() * 9, contentY + 12);
  display.print(statusText);
  
  contentY += 60;
  
  // Service details
  display.setTextSize(2);
  display.setTextColor(TFT_YELLOW, TFT_BLACK);
  display.setCursor(10, contentY);
  display.printf("Type: %s", getServiceTypeString(svc.type).c_str());
  contentY += 25;
  
  // Host/URL information
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  if (svc.type == TYPE_HTTP_GET && svc.url.length() > 0) {
    display.setCursor(10, contentY);
    display.print("URL:");
    contentY += 20;
    display.setTextSize(1);
    String urlDisplay = svc.url;
    // Wrap long URLs across multiple lines
    int lines = (urlDisplay.length() + URL_CHARS_PER_LINE - 1) / URL_CHARS_PER_LINE;
    for (int i = 0; i < lines && i < URL_MAX_LINES; i++) {
      display.setCursor(10, contentY);
      display.print(urlDisplay.substring(i * URL_CHARS_PER_LINE, min((i + 1) * URL_CHARS_PER_LINE, (int)urlDisplay.length())));
      contentY += 12;
    }
    display.setTextSize(2);
  } else if (svc.type == TYPE_PUSH) {
    display.setCursor(10, contentY);
    display.print("Push-based monitor");
    contentY += 25;
  } else if (svc.type == TYPE_PING) {
    display.setCursor(10, contentY);
    display.printf("Host: %s", svc.host.c_str());
    contentY += 25;
  } else {
    display.setCursor(10, contentY);
    display.printf("Host: %s:%d", svc.host.c_str(), svc.port);
    contentY += 25;
  }
  
  // SNMP-specific info
  if (svc.type == TYPE_SNMP_GET && svc.snmpOid.length() > 0) {
    display.setCursor(10, contentY);
    display.setTextSize(1);
    display.printf("OID: %s", svc.snmpOid.c_str());
    contentY += 15;
    display.setTextSize(2);
  }
  
  contentY += 10;
  
  // Timing information
  display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  display.setCursor(10, contentY);
  display.printf("Interval: %ds", svc.checkInterval);
  contentY += 25;
  
  display.setCursor(10, contentY);
  if (svc.lastCheck == 0) {
    display.print("Last check: pending");
  } else {
    unsigned long sinceCheck = (millis() - svc.lastCheck) / 1000;
    display.printf("Last check: %lus ago", sinceCheck);
  }
  contentY += 25;
  
  // Thresholds
  display.setCursor(10, contentY);
  display.printf("Thresholds: %d fail / %d pass", svc.failThreshold, svc.passThreshold);
  contentY += 25;
  
  // Consecutive counts
  display.setCursor(10, contentY);
  display.printf("Consecutive: %d pass / %d fail", svc.consecutivePasses, svc.consecutiveFails);
  contentY += 25;
  
  // Pause status
  if (svc.pauseUntil > 0) {
    unsigned long remaining = getPauseRemainingMs(svc.pauseUntil, millis());
    if (remaining > 0) {
      display.setTextColor(TFT_ORANGE, TFT_BLACK);
      display.setCursor(10, contentY);
      display.printf("Paused: %lus remaining", remaining / 1000);
      contentY += 25;
    }
  }
  
  // Error message if present
  if (svc.lastError.length() > 0) {
    display.setTextColor(TFT_RED, TFT_BLACK);
    display.setCursor(10, contentY);
    display.print("Error:");
    contentY += 20;
    display.setTextSize(1);
    String errorDisplay = svc.lastError;
    if (errorDisplay.length() > 50) {
      errorDisplay = errorDisplay.substring(0, 47) + "...";
    }
    display.setCursor(10, contentY);
    display.print(errorDisplay);
  }
  display.endWrite();
}

// Handle touch input for main view
void handleMainViewTouch(int16_t x, int16_t y) {
  int16_t width = display.width();
  int16_t height = display.height();
  
  // Check power button (top right) - simple tap to turn off screen
  int powerBtnX = width - POWER_BUTTON_SIZE - 5;
  int powerBtnY = 5;
  if (x >= powerBtnX && x <= powerBtnX + POWER_BUTTON_SIZE &&
      y >= powerBtnY && y <= powerBtnY + POWER_BUTTON_SIZE) {
    turnScreenOff();
    return;
  }
  
  // Check service buttons
  if (serviceCount > 0 && y > HEADER_HEIGHT) {
    int cols = 2;
    int buttonWidth = (width - (cols + 1) * SERVICE_BUTTON_MARGIN) / cols;
    int startY = HEADER_HEIGHT + SERVICE_BUTTON_MARGIN;
    int availableHeight = height - startY - SERVICE_BUTTON_MARGIN;
    int maxRows = availableHeight / (SERVICE_BUTTON_HEIGHT + SERVICE_BUTTON_MARGIN);
    
    for (int i = 0; i < serviceCount && i < maxRows * cols; i++) {
      int row = i / cols;
      int col = i % cols;
      
      int btnX = SERVICE_BUTTON_MARGIN + col * (buttonWidth + SERVICE_BUTTON_MARGIN);
      int btnY = startY + row * (SERVICE_BUTTON_HEIGHT + SERVICE_BUTTON_MARGIN);
      
      if (x >= btnX && x <= btnX + buttonWidth &&
          y >= btnY && y <= btnY + SERVICE_BUTTON_HEIGHT) {
        // Service button tapped - show detail view
        currentServiceIndex = i;
        currentView = VIEW_DETAIL;
        displayNeedsUpdate = true;
        return;
      }
    }
  }
}

// Handle touch input for detail view
void handleDetailViewTouch(int16_t x, int16_t y) {
  // Check back button (top left)
  if (x >= 5 && x <= 65 && y >= 5 && y <= 5 + POWER_BUTTON_SIZE) {
    currentView = VIEW_MAIN;
    displayNeedsUpdate = true;
    return;
  }
}

// Handle touch input when screen is off (double-tap to wake)
void handleScreenOffTouch(int16_t x, int16_t y) {
  unsigned long now = millis();
  
  // Check for double-tap
  if (now - lastTapTime <= DOUBLE_TAP_WINDOW_MS) {
    // Double-tap detected - wake up screen
    turnScreenOn();
    lastTapTime = 0;  // Reset to prevent accidental triple-tap
  } else {
    lastTapTime = now;
  }
}

void handleDisplayLoop() {
  if (!displayReady) return;

  unsigned long now = millis();

  // Handle touch input
  if (touchReady && (now - lastTouchTime >= TOUCH_DEBOUNCE_MS)) {
    lgfx::touch_point_t tp;
    int touchCount = display.getTouch(&tp, 1);
    
    if (touchCount > 0) {
      int16_t x = tp.x;
      int16_t y = tp.y;
      lastTouchTime = now;
      
      if (currentView == VIEW_OFF) {
        // Screen is off - check for double-tap to wake
        handleScreenOffTouch(x, y);
      } else {
        // Screen is on - record activity and handle touch
        recordActivity();
        
        switch (currentView) {
          case VIEW_MAIN:
            handleMainViewTouch(x, y);
            break;
          case VIEW_DETAIL:
            handleDetailViewTouch(x, y);
            break;
          default:
            break;
        }
      }
    }
  }
  
  // Handle screen timeout (only when screen is on and timeout is enabled)
  if (currentView != VIEW_OFF && screenTimeoutMs > 0) {
    if (now - lastActivityTime >= screenTimeoutMs) {
      turnScreenOff();
    }
  }

  // Update display if needed
  if (displayNeedsUpdate && currentView != VIEW_OFF) {
    switch (currentView) {
      case VIEW_MAIN:
        renderMainView();
        break;
      case VIEW_DETAIL:
        renderDetailView();
        break;
      default:
        break;
    }
    displayNeedsUpdate = false;
  }
  
  // Periodically refresh display to update status changes
  // Auto-refresh applies to both main view (all services) and detail view (single service)
  static unsigned long lastAutoRefresh = 0;
  if ((currentView == VIEW_MAIN || currentView == VIEW_DETAIL) && 
      now - lastAutoRefresh >= DISPLAY_AUTO_REFRESH_MS) {
    displayNeedsUpdate = true;
    lastAutoRefresh = now;
  }
}

#endif // HAS_LCD

String getWebPage() {
  return R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Uptime Monitor - Status</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { text-align: center; color: white; margin-bottom: 30px; }
        .header h1 { font-size: 2.5em; margin-bottom: 10px; text-shadow: 2px 2px 4px rgba(0,0,0,0.2); }
        .header p { font-size: 1.1em; opacity: 0.9; }
        .admin-link { text-align: center; margin-bottom: 20px; }
        .admin-link a {
            display: inline-block; padding: 12px 24px; background: white; color: #667eea;
            text-decoration: none; border-radius: 6px; font-weight: 600; transition: all 0.3s;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        .admin-link a:hover { transform: translateY(-2px); box-shadow: 0 6px 12px rgba(0,0,0,0.15); }
        .status-table {
            background: white; border-radius: 12px; padding: 25px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1); overflow-x: auto;
        }
        table { width: 100%; border-collapse: collapse; }
        th {
            background: #f9fafb; padding: 15px; text-align: left; font-weight: 600;
            color: #374151; border-bottom: 2px solid #e5e7eb;
        }
        td { padding: 15px; border-bottom: 1px solid #e5e7eb; color: #6b7280; }
        tr:last-child td { border-bottom: none; }
        tr:hover { background: #f9fafb; }
        .service-name { font-weight: 600; color: #1f2937; }
        .status-badge {
            display: inline-block; padding: 4px 12px; border-radius: 20px;
            font-size: 0.85em; font-weight: 600;
        }
        .status-badge.up { background: #d1fae5; color: #065f46; }
        .status-badge.down { background: #fee2e2; color: #991b1b; }
        .status-badge.pending { background: #e0e7ff; color: #3730a3; }
        .status-badge.paused { background: #fef3c7; color: #92400e; }
        .empty-state { text-align: center; padding: 60px 20px; color: white; }
        .empty-state h3 { font-size: 1.5em; margin-bottom: 10px; }
        .hidden { display: none; }
        @media (max-width: 768px) {
            .header h1 { font-size: 1.8em; }
            .status-table { padding: 15px; }
            th, td { padding: 10px 8px; font-size: 0.9em; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>ESP32 Uptime Monitor</h1>
            <p>Service Status Overview</p>
        </div>
        <div class="admin-link">
            <a href="/admin">Administration Panel</a>
        </div>
        <div class="status-table" id="statusTable">
            <table>
                <thead>
                    <tr>
                        <th>Service</th>
                        <th>Type</th>
                        <th>Status</th>
                        <th>Last Checked</th>
                    </tr>
                </thead>
                <tbody id="servicesTableBody">
                </tbody>
            </table>
        </div>
        <div id="emptyState" class="empty-state hidden">
            <h3>No services configured</h3>
            <p>Visit the <a href="/admin" style="color: white; text-decoration: underline;">administration panel</a> to add services</p>
        </div>
    </div>
    <script>
        let services = [];
        async function loadServices() {
            try {
                const response = await fetch('/api/services');
                const data = await response.json();
                services = data.services || [];
                // Sort services alphabetically by name
                services.sort((a, b) => a.name.localeCompare(b.name));
                renderServices();
            } catch (error) {
                console.error('Error loading services:', error);
            }
        }
        function renderServices() {
            const tbody = document.getElementById('servicesTableBody');
            const table = document.getElementById('statusTable');
            const emptyState = document.getElementById('emptyState');
            if (services.length === 0) {
                table.classList.add('hidden');
                emptyState.classList.remove('hidden');
                return;
            }
            table.classList.remove('hidden');
            emptyState.classList.add('hidden');
            tbody.innerHTML = services.map(service => {
                let uptimeStr = 'Not checked yet';
                if (service.secondsSinceLastCheck >= 0) {
                    const seconds = service.secondsSinceLastCheck;
                    if (seconds < 60) {
                        uptimeStr = `${seconds}s ago`;
                    } else if (seconds < 3600) {
                        const minutes = Math.floor(seconds / 60);
                        const secs = seconds % 60;
                        uptimeStr = `${minutes}m ${secs}s ago`;
                    } else {
                        const hours = Math.floor(seconds / 3600);
                        const minutes = Math.floor((seconds % 3600) / 60);
                        uptimeStr = `${hours}h ${minutes}m ago`;
                    }
                }
                const isPending = service.secondsSinceLastCheck < 0;
                let statusText = service.isUp ? 'UP' : 'DOWN';
                let statusClass = service.isUp ? 'up' : 'down';
                if (isPending) {
                    statusText = 'PENDING';
                    statusClass = 'pending';
                } else if (!service.enabled) {
                    statusText = 'DISABLED';
                    statusClass = 'paused';
                } else if (service.pauseRemaining > 0) {
                    const pauseMins = Math.floor(service.pauseRemaining / 60);
                    const pauseSecs = service.pauseRemaining % 60;
                    const pauseStr = pauseMins > 0 ? `${pauseMins}m ${pauseSecs}s` : `${pauseSecs}s`;
                    statusText = `PAUSED (${pauseStr})`;
                    statusClass = 'paused';
                }
                // Format service type for display
                const typeDisplay = service.type.replace('_', ' ').toUpperCase();
                return `
                    <tr>
                        <td class="service-name">${service.name}</td>
                        <td>${typeDisplay}</td>
                        <td><span class="status-badge ${statusClass}">${statusText}</span></td>
                        <td>${uptimeStr}</td>
                    </tr>
                `;
            }).join('');
        }
        setInterval(loadServices, 5000);
        loadServices();
    </script>
</body>
</html>)rawliteral";
}
String getAdminPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Uptime Monitor - Admin</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }

        .container {
            max-width: 1200px;
            margin: 0 auto;
        }

        .header {
            text-align: center;
            color: white;
            margin-bottom: 30px;
        }

        .header h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.2);
        }

        .header p {
            font-size: 1.1em;
            opacity: 0.9;
        }

        .card {
            background: white;
            border-radius: 12px;
            padding: 25px;
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }

        .add-service-form {
            display: grid;
            gap: 15px;
        }

        .form-group {
            display: flex;
            flex-direction: column;
        }

        .form-row {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }

        label {
            font-weight: 600;
            margin-bottom: 5px;
            color: #333;
            font-size: 0.9em;
        }

        input, select {
            padding: 10px;
            border: 2px solid #e0e0e0;
            border-radius: 6px;
            font-size: 1em;
            transition: border-color 0.3s;
        }

        input:focus, select:focus {
            outline: none;
            border-color: #667eea;
        }

        .btn {
            padding: 12px 24px;
            border: none;
            border-radius: 6px;
            font-size: 1em;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s;
        }

        .btn-primary {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }

        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
        }

        .btn-danger {
            background: #ef4444;
            color: white;
            padding: 8px 16px;
            font-size: 0.9em;
        }

        .btn-danger:hover {
            background: #dc2626;
        }

        .btn-secondary {
            background: #6b7280;
            color: white;
        }

        .btn-secondary:hover {
            background: #4b5563;
        }

        .card-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            gap: 12px;
            margin-bottom: 20px;
        }

        .backup-actions {
            display: flex;
            gap: 10px;
            align-items: center;
            flex-wrap: wrap;
        }

        .backup-actions input[type="file"] {
            display: none;
        }

        .backup-actions .btn {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            height: 44px;
            padding: 0 18px;
            line-height: 1;
            box-sizing: border-box;
        }

        .services-table {
            background: white;
            border-radius: 12px;
            padding: 25px;
            margin-top: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            overflow-x: auto;
        }

        .services-table table {
            width: 100%;
            border-collapse: collapse;
        }

        .services-table th {
            background: #f9fafb;
            padding: 12px 15px;
            text-align: left;
            font-weight: 600;
            color: #374151;
            border-bottom: 2px solid #e5e7eb;
            font-size: 0.9em;
        }

        .services-table td {
            padding: 12px 15px;
            border-bottom: 1px solid #e5e7eb;
            color: #6b7280;
            font-size: 0.9em;
        }

        .services-table tr:last-child td {
            border-bottom: none;
        }

        .services-table tr:hover {
            background: #f9fafb;
        }

        .services-table .service-name-cell {
            font-weight: 600;
            color: #1f2937;
        }

        .services-table .status-badge {
            display: inline-block;
            padding: 4px 10px;
            border-radius: 12px;
            font-size: 0.8em;
            font-weight: 600;
        }

        .services-table .status-badge.up {
            background: #d1fae5;
            color: #065f46;
        }

        .services-table .status-badge.down {
            background: #fee2e2;
            color: #991b1b;
        }

        .services-table .status-badge.pending {
            background: #e0e7ff;
            color: #3730a3;
        }

        .services-table .status-badge.paused {
            background: #fef3c7;
            color: #92400e;
        }

        .services-table .btn-group {
            display: flex;
            gap: 5px;
            flex-wrap: wrap;
        }

        .services-table .btn-small {
            padding: 6px 12px;
            font-size: 0.85em;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            transition: all 0.2s;
            font-weight: 500;
        }

        .services-table .btn-edit {
            background: #3b82f6;
            color: white;
        }

        .services-table .btn-edit:hover {
            background: #2563eb;
        }

        .services-table .btn-pause {
            background: #f59e0b;
            color: white;
        }

        .services-table .btn-pause:hover {
            background: #d97706;
        }

        .services-table .btn-disable {
            background: #6b7280;
            color: white;
        }

        .services-table .btn-disable:hover {
            background: #4b5563;
        }

        .services-table .btn-enable {
            background: #10b981;
            color: white;
        }

        .services-table .btn-enable:hover {
            background: #059669;
        }

        .services-table .btn-delete {
            background: #ef4444;
            color: white;
        }

        .services-table .btn-delete:hover {
            background: #dc2626;
        }


        .services-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
            gap: 20px;
        }

        .service-card {
            background: white;
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            border-left: 4px solid #e0e0e0;
            transition: all 0.3s;
        }

        .service-card.up {
            border-left-color: #10b981;
        }

        .service-card.down {
            border-left-color: #ef4444;
        }

        .service-card.paused {
            border-left-color: #f59e0b;
            opacity: 0.8;
        }

        .service-card.pending {
            border-left-color: #6366f1;
        }

        .service-card:hover {
            transform: translateY(-4px);
            box-shadow: 0 4px 12px rgba(0,0,0,0.15);
        }

        .service-header {
            display: flex;
            justify-content: space-between;
            align-items: start;
            margin-bottom: 15px;
        }

        .service-name {
            font-size: 1.2em;
            font-weight: 700;
            color: #1f2937;
        }

        .service-status {
            display: inline-block;
            padding: 4px 12px;
            border-radius: 20px;
            font-size: 0.85em;
            font-weight: 600;
        }

        .service-status.up {
            background: #d1fae5;
            color: #065f46;
        }

        .service-status.down {
            background: #fee2e2;
            color: #991b1b;
        }

        .service-status.pending {
            background: #e0e7ff;
            color: #3730a3;
        }

        .service-info {
            margin-bottom: 10px;
            color: #6b7280;
            font-size: 0.9em;
        }

        .service-info strong {
            color: #374151;
        }

        .service-actions {
            margin-top: 15px;
            padding-top: 15px;
            border-top: 1px solid #e5e7eb;
            display: flex;
            flex-wrap: wrap;
            gap: 8px;
        }

        .modal-overlay {
            position: fixed;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: rgba(0,0,0,0.5);
            display: flex;
            align-items: center;
            justify-content: center;
            z-index: 1000;
        }

        .modal {
            background: white;
            border-radius: 12px;
            padding: 25px;
            max-width: 400px;
            width: 90%;
        }

        .modal h3 {
            margin-bottom: 15px;
            color: #1f2937;
        }

        .modal-actions {
            display: flex;
            gap: 10px;
            margin-top: 20px;
            justify-content: flex-end;
        }

        .pause-options {
            display: flex;
            flex-direction: column;
            gap: 10px;
        }

        .pause-options button {
            width: 100%;
        }

        .type-badge {
            display: inline-block;
            padding: 4px 10px;
            background: #e0e7ff;
            color: #3730a3;
            border-radius: 6px;
            font-size: 0.8em;
            font-weight: 600;
            margin-bottom: 10px;
        }

        .empty-state {
            text-align: center;
            padding: 60px 20px;
            color: white;
        }

        .empty-state h3 {
            font-size: 1.5em;
            margin-bottom: 10px;
        }

        .hidden {
            display: none;
        }

        .alert {
            padding: 12px 20px;
            border-radius: 6px;
            margin-bottom: 20px;
        }

        .alert-success {
            background: #d1fae5;
            color: #065f46;
        }

        .alert-error {
            background: #fee2e2;
            color: #991b1b;
        }

        @media (max-width: 768px) {
            .form-row {
                grid-template-columns: 1fr;
            }

            .services-grid {
                grid-template-columns: 1fr;
            }

            .header h1 {
                font-size: 1.8em;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>ESP32 Uptime Monitor - Admin</h1>
            <p><a href="/" style="color: white; text-decoration: underline; opacity: 0.9;">← Back to Status View</a></p>
        </div>

        <div id="alertContainer"></div>

        <div class="card">
            <div class="card-header">
                <h2 style="margin: 0; color: #1f2937;">Add New Service</h2>
                <div class="backup-actions">
                    <a href="/update" class="btn btn-secondary" target="_blank" rel="noopener noreferrer" title="Open firmware update page">OTA Update</a>
                    <button type="button" class="btn btn-secondary" onclick="exportServices()">Export Monitors</button>
                    <label class="btn btn-secondary" for="importFile">Import Monitors</label>
                    <input type="file" id="importFile" accept=".json" onchange="importServices(this.files[0])">
                </div>
            </div>
            <form id="addServiceForm" class="add-service-form">
                <div class="form-group">
                    <label for="serviceName">Service Name</label>
                    <input type="text" id="serviceName" required placeholder="My Service">
                </div>

                <div class="form-row">
                    <div class="form-group">
                        <label for="serviceType">Service Type</label>
                        <select id="serviceType" required>
                            <option value="http_get">HTTP GET</option>
                            <option value="ping">Ping</option>
                            <option value="snmp_get">SNMP GET</option>
                            <option value="port">Port Check</option>
                            <option value="push">Push</option>
                        </select>
                    </div>

                    <div class="form-group" id="hostGroup">
                        <label for="serviceHost">Host / IP Address</label>
                        <input type="text" id="serviceHost" placeholder="192.168.1.100">
                    </div>
                </div>

                <div class="form-group" id="urlGroup">
                    <label for="serviceUrl">URL (http:// or https://)</label>
                    <input type="url" id="serviceUrl" placeholder="https://example.com/health" title="Full URL including protocol (http:// or https://)">
                </div>

                <div class="form-row">
                    <div class="form-group" id="portGroup">
                        <label for="servicePort">Port</label>
                        <input type="number" id="servicePort" value="80" required>
                    </div>

                    <div class="form-group">
                        <label for="checkInterval">Check Interval (seconds)</label>
                        <input type="number" id="checkInterval" value="60" required min="10">
                    </div>
                </div>

                <div class="form-row">
                    <div class="form-group">
                        <label for="failThreshold">Fail Threshold</label>
                        <input type="number" id="failThreshold" value="3" required min="1" title="Number of consecutive failures before marking as DOWN">
                    </div>

                    <div class="form-group">
                        <label for="passThreshold">Pass Threshold</label>
                        <input type="number" id="passThreshold" value="1" required min="1" title="Number of consecutive successes before marking as UP">
                    </div>
                </div>

                <div class="form-group">
                    <label for="rearmCount">Re-arm Alert Count (0 = disabled)</label>
                    <input type="number" id="rearmCount" value="1440" required min="0" title="Number of failed checks before re-alerting while service is DOWN. Set to 0 to disable.">
                </div>

                <div class="form-group hidden" id="pathGroup">
                    <label for="servicePath">Path</label>
                    <input type="text" id="servicePath" value="/" placeholder="/">
                </div>

                <div class="form-group" id="responseGroup">
                    <label for="expectedResponse">Expected Response (* for any, regex: prefix for regex)</label>
                    <input type="text" id="expectedResponse" value="*" placeholder="*" title="Use * for any response, plain text for substring match, or regex:pattern for regex matching (e.g., regex:status.*ok)">
                </div>

                <div class="form-group hidden" id="snmpOidGroup">
                    <label for="snmpOid">SNMP OID</label>
                    <input type="text" id="snmpOid" value="" placeholder="1.3.6.1.2.1.1.1.0">
                </div>

                <div class="form-group hidden" id="snmpCommunityGroup">
                    <label for="snmpCommunity">SNMP Community String</label>
                    <input type="text" id="snmpCommunity" value="public" placeholder="public">
                </div>

                <div class="form-row hidden" id="snmpCompareGroup">
                    <div class="form-group">
                        <label for="snmpCompareOp">Comparison Operator</label>
                        <select id="snmpCompareOp">
                            <option value="=">=  (Equal)</option>
                            <option value="<>"><>  (Not Equal)</option>
                            <option value="<"><  (Less Than)</option>
                            <option value="<="><= (Less or Equal)</option>
                            <option value=">">  (Greater Than)</option>
                            <option value=">=">>= (Greater or Equal)</option>
                        </select>
                    </div>

                    <div class="form-group">
                        <label for="snmpExpectedValue">Expected Value</label>
                        <input type="text" id="snmpExpectedValue" value="" placeholder="Expected value">
                    </div>
                </div>

                <button type="submit" class="btn btn-primary">Add Service</button>
            </form>
        </div>

        <div class="card">
            <h2 style="margin: 0 0 20px 0; color: #1f2937;">Monitored Services</h2>
            <div class="services-table">
                <table>
                    <thead>
                        <tr>
                            <th>Service Name</th>
                            <th>Type</th>
                            <th>Target</th>
                            <th>Status</th>
                            <th>Last Check</th>
                            <th>Actions</th>
                        </tr>
                    </thead>
                    <tbody id="servicesTableBody">
                    </tbody>
                </table>
            </div>
        </div>
        <div id="emptyState" class="empty-state hidden">
            <h3>No services yet</h3>
            <p>Add your first service using the form above</p>
        </div>
    </div>

    <script>
        let services = [];
        let editingPushToken = null;  // Preserve pushToken when editing PUSH services

        // Update form fields based on service type
        document.getElementById('serviceType').addEventListener('change', function() {
            const type = this.value;
            const hostGroup = document.getElementById('hostGroup');
            const hostInput = document.getElementById('serviceHost');
            const urlGroup = document.getElementById('urlGroup');
            const urlInput = document.getElementById('serviceUrl');
            const pathGroup = document.getElementById('pathGroup');
            const responseGroup = document.getElementById('responseGroup');
            const portGroup = document.getElementById('portGroup');
            const portInput = document.getElementById('servicePort');
            const snmpOidGroup = document.getElementById('snmpOidGroup');
            const snmpCommunityGroup = document.getElementById('snmpCommunityGroup');
            const snmpCompareGroup = document.getElementById('snmpCompareGroup');

            if (type === 'http_get') {
                // HTTP GET uses URL field only
                hostGroup.classList.add('hidden');
                hostInput.removeAttribute('required');
                urlGroup.classList.remove('hidden');
                urlInput.setAttribute('required', '');
                portGroup.classList.add('hidden');
                pathGroup.classList.add('hidden');
                responseGroup.classList.remove('hidden');
                snmpOidGroup.classList.add('hidden');
                snmpCommunityGroup.classList.add('hidden');
                snmpCompareGroup.classList.add('hidden');
            } else if (type === 'push') {
                // Push type doesn't need host/port/path/url
                hostGroup.classList.add('hidden');
                hostInput.removeAttribute('required');
                urlGroup.classList.add('hidden');
                urlInput.removeAttribute('required');
                portGroup.classList.add('hidden');
                pathGroup.classList.add('hidden');
                responseGroup.classList.add('hidden');
                snmpOidGroup.classList.add('hidden');
                snmpCommunityGroup.classList.add('hidden');
                snmpCompareGroup.classList.add('hidden');
            } else if (type === 'ping') {
                hostGroup.classList.remove('hidden');
                hostInput.setAttribute('required', '');
                urlGroup.classList.add('hidden');
                urlInput.removeAttribute('required');
                portGroup.classList.add('hidden');
                pathGroup.classList.add('hidden');
                responseGroup.classList.add('hidden');
                snmpOidGroup.classList.add('hidden');
                snmpCommunityGroup.classList.add('hidden');
                snmpCompareGroup.classList.add('hidden');
            } else if (type === 'port') {
                hostGroup.classList.remove('hidden');
                hostInput.setAttribute('required', '');
                urlGroup.classList.add('hidden');
                urlInput.removeAttribute('required');
                portGroup.classList.remove('hidden');
                pathGroup.classList.add('hidden');
                responseGroup.classList.add('hidden');
                snmpOidGroup.classList.add('hidden');
                snmpCommunityGroup.classList.add('hidden');
                snmpCompareGroup.classList.add('hidden');
                portInput.value = 22;
            } else if (type === 'snmp_get') {
                hostGroup.classList.remove('hidden');
                hostInput.setAttribute('required', '');
                urlGroup.classList.add('hidden');
                urlInput.removeAttribute('required');
                portGroup.classList.remove('hidden');
                pathGroup.classList.add('hidden');
                responseGroup.classList.add('hidden');
                snmpOidGroup.classList.remove('hidden');
                snmpCommunityGroup.classList.remove('hidden');
                snmpCompareGroup.classList.remove('hidden');
                portInput.value = 161;
            }
        });

        // Add service
        document.getElementById('addServiceForm').addEventListener('submit', async function(e) {
            e.preventDefault();

            const data = {
                name: document.getElementById('serviceName').value,
                type: document.getElementById('serviceType').value,
                host: document.getElementById('serviceHost').value,
                port: parseInt(document.getElementById('servicePort').value) || 80,
                path: document.getElementById('servicePath').value,
                url: document.getElementById('serviceUrl').value,
                expectedResponse: document.getElementById('expectedResponse').value,
                checkInterval: parseInt(document.getElementById('checkInterval').value),
                passThreshold: parseInt(document.getElementById('passThreshold').value),
                failThreshold: parseInt(document.getElementById('failThreshold').value),
                rearmCount: parseInt(document.getElementById('rearmCount').value),
                snmpOid: document.getElementById('snmpOid').value,
                snmpCommunity: document.getElementById('snmpCommunity').value,
                snmpCompareOp: document.getElementById('snmpCompareOp').value,
                snmpExpectedValue: document.getElementById('snmpExpectedValue').value
            };

            // Preserve pushToken when editing a PUSH service
            if (editingPushToken && data.type === 'push') {
                data.pushToken = editingPushToken;
            }

            try {
                const response = await fetch('/api/services', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });

                if (response.ok) {
                    showAlert('Service added successfully!', 'success');
                    this.reset();
                    editingPushToken = null;  // Clear the stored pushToken
                    document.getElementById('serviceType').dispatchEvent(new Event('change'));
                    loadServices();
                } else {
                    showAlert('Failed to add service', 'error');
                }
            } catch (error) {
                showAlert('Error: ' + error.message, 'error');
            }
        });

        // Load services
        async function loadServices() {
            try {
                const response = await fetch('/api/services');
                const data = await response.json();
                services = data.services || [];
                renderServices();
            } catch (error) {
                console.error('Error loading services:', error);
            }
        }

        // Render services
        function renderServices() {
            const tbody = document.getElementById('servicesTableBody');
            const emptyState = document.getElementById('emptyState');

            if (services.length === 0) {
                tbody.innerHTML = '<tr><td colspan="6" style="text-align: center; padding: 40px; color: #9ca3af;">No services configured yet. Add your first service using the form above.</td></tr>';
                return;
            }

            emptyState.classList.add('hidden');

            tbody.innerHTML = services.map(service => {
                let uptimeStr = 'Not checked yet';

                if (service.secondsSinceLastCheck >= 0) {
                    const seconds = service.secondsSinceLastCheck;
                    if (seconds < 60) {
                        uptimeStr = `${seconds}s ago`;
                    } else if (seconds < 3600) {
                        const minutes = Math.floor(seconds / 60);
                        const secs = seconds % 60;
                        uptimeStr = `${minutes}m ${secs}s ago`;
                    } else {
                        const hours = Math.floor(seconds / 3600);
                        const minutes = Math.floor((seconds % 3600) / 60);
                        uptimeStr = `${hours}h ${minutes}m ago`;
                    }
                }

                // Determine status
                const isPending = service.secondsSinceLastCheck < 0;
                let statusText = service.isUp ? 'UP' : 'DOWN';
                let statusClass = service.isUp ? 'up' : 'down';
                
                if (isPending) {
                    statusText = 'PENDING';
                    statusClass = 'pending';
                } else if (!service.enabled) {
                    statusText = 'DISABLED';
                    statusClass = 'paused';
                } else if (service.pauseRemaining > 0) {
                    const pauseMins = Math.floor(service.pauseRemaining / 60);
                    const pauseSecs = service.pauseRemaining % 60;
                    const pauseStr = pauseMins > 0 ? `${pauseMins}m ${pauseSecs}s` : `${pauseSecs}s`;
                    statusText = `PAUSED (${pauseStr})`;
                    statusClass = 'paused';
                }

                // Build target info based on service type
                let target = '';
                if (service.type === 'http_get' && service.url) {
                    target = service.url;
                } else if (service.type === 'push') {
                    target = 'Push endpoint';
                } else if (service.type === 'ping') {
                    target = service.host;
                } else if (service.host) {
                    target = `${service.host}:${service.port}`;
                }

                // Build action buttons
                const editBtn = `<button class="btn-small btn-edit" onclick="editService('${service.id}')">Edit</button>`;
                const pauseBtn = service.pauseRemaining > 0
                    ? `<button class="btn-small btn-pause" onclick="pauseService('${service.id}', 0)">Unpause</button>`
                    : `<button class="btn-small btn-pause" onclick="showPauseDialog('${service.id}')">Pause</button>`;
                const enableBtn = service.enabled
                    ? `<button class="btn-small btn-disable" onclick="toggleService('${service.id}', false)">Disable</button>`
                    : `<button class="btn-small btn-enable" onclick="toggleService('${service.id}', true)">Enable</button>`;
                const deleteBtn = `<button class="btn-small btn-delete" onclick="deleteService('${service.id}')">Delete</button>`;

                return `
                    <tr>
                        <td class="service-name-cell">${service.name}</td>
                        <td>${service.type.replace('_', ' ').toUpperCase()}</td>
                        <td style="word-break: break-all; max-width: 300px;">${target}</td>
                        <td><span class="status-badge ${statusClass}">${statusText}</span></td>
                        <td>${uptimeStr}</td>
                        <td>
                            <div class="btn-group">
                                ${editBtn}
                                ${pauseBtn}
                                ${enableBtn}
                                ${deleteBtn}
                            </div>
                        </td>
                    </tr>
                `;
            }).join('');
        }

                // Delete service
        async function deleteService(id) {
            if (!confirm('Are you sure you want to delete this service?')) {
                return;
            }

            try {
                const response = await fetch(`/api/services/${id}`, {
                    method: 'DELETE'
                });

                if (response.ok) {
                    showAlert('Service deleted successfully', 'success');
                    loadServices();
                } else {
                    showAlert('Failed to delete service', 'error');
                }
            } catch (error) {
                showAlert('Error: ' + error.message, 'error');
            }
        }

        // Edit service - loads values into form and deletes the old service
        async function editService(id) {
            const service = services.find(s => s.id === id);
            if (!service) {
                showAlert('Service not found', 'error');
                return;
            }

            // Confirm before editing (since the old service will be deleted)
            if (!confirm('Edit this service? The current configuration will be loaded into the form for modification.')) {
                return;
            }

            // Preserve pushToken for PUSH services so URL doesn't change
            if (service.type === 'push' && service.pushToken) {
                editingPushToken = service.pushToken;
            } else {
                editingPushToken = null;
            }

            // Populate form with existing values
            document.getElementById('serviceName').value = service.name;
            document.getElementById('serviceType').value = service.type;
            document.getElementById('serviceHost').value = service.host || '';
            document.getElementById('servicePort').value = service.port || 80;
            document.getElementById('servicePath').value = service.path || '/';
            document.getElementById('serviceUrl').value = service.url || '';
            document.getElementById('expectedResponse').value = service.expectedResponse || '*';
            document.getElementById('checkInterval').value = service.checkInterval || 60;
            document.getElementById('passThreshold').value = service.passThreshold || 1;
            document.getElementById('failThreshold').value = service.failThreshold || 3;
            document.getElementById('rearmCount').value = (service.rearmCount !== undefined ? service.rearmCount : 1440);
            document.getElementById('snmpOid').value = service.snmpOid || '';
            document.getElementById('snmpCommunity').value = service.snmpCommunity || 'public';
            document.getElementById('snmpCompareOp').value = service.snmpCompareOp || '=';
            document.getElementById('snmpExpectedValue').value = service.snmpExpectedValue || '';

            // Trigger change event to show/hide appropriate fields
            document.getElementById('serviceType').dispatchEvent(new Event('change'));

            // Scroll to form
            document.getElementById('addServiceForm').scrollIntoView({ behavior: 'smooth' });

            // Delete the old service
            try {
                const response = await fetch(`/api/services/${id}`, {
                    method: 'DELETE'
                });

                if (response.ok) {
                    showAlert('Service loaded for editing. Make your changes and click "Add Service" to save.', 'success');
                    loadServices();
                } else {
                    showAlert('Failed to load service for editing', 'error');
                    editingPushToken = null;  // Clear on failure
                }
            } catch (error) {
                showAlert('Error: ' + error.message, 'error');
                editingPushToken = null;  // Clear on failure
            }
        }

        // Toggle service enabled/disabled
        async function toggleService(id, enabled) {
            try {
                const response = await fetch(`/api/services/${id}`, {
                    method: 'PATCH',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ enabled: enabled })
                });

                if (response.ok) {
                    showAlert(`Service ${enabled ? 'enabled' : 'disabled'} successfully`, 'success');
                    loadServices();
                } else {
                    showAlert('Failed to update service', 'error');
                }
            } catch (error) {
                showAlert('Error: ' + error.message, 'error');
            }
        }

        // Pause service for specified duration
        async function pauseService(id, durationSeconds) {
            try {
                const response = await fetch(`/api/services/${id}`, {
                    method: 'PATCH',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ pauseDuration: durationSeconds })
                });

                if (response.ok) {
                    if (durationSeconds > 0) {
                        const mins = Math.floor(durationSeconds / 60);
                        const secs = durationSeconds % 60;
                        const timeStr = mins > 0 ? `${mins} minute(s)` : `${secs} seconds`;
                        showAlert(`Service paused for ${timeStr}`, 'success');
                    } else {
                        showAlert('Service unpaused', 'success');
                    }
                    loadServices();
                    closePauseDialog();
                } else {
                    showAlert('Failed to update service', 'error');
                }
            } catch (error) {
                showAlert('Error: ' + error.message, 'error');
            }
        }

        // Show pause duration dialog
        let currentPauseServiceId = null;
        function showPauseDialog(id) {
            currentPauseServiceId = id;
            const modal = document.createElement('div');
            modal.className = 'modal-overlay';
            modal.id = 'pauseModal';
            modal.innerHTML = `
                <div class="modal">
                    <h3>Pause Service Checks</h3>
                    <div class="pause-options">
                        <button class="btn btn-secondary" onclick="pauseService('${id}', 300)">5 minutes</button>
                        <button class="btn btn-secondary" onclick="pauseService('${id}', 900)">15 minutes</button>
                        <button class="btn btn-secondary" onclick="pauseService('${id}', 1800)">30 minutes</button>
                        <button class="btn btn-secondary" onclick="pauseService('${id}', 3600)">1 hour</button>
                        <button class="btn btn-secondary" onclick="pauseService('${id}', 14400)">4 hours</button>
                        <button class="btn btn-secondary" onclick="pauseService('${id}', 86400)">24 hours</button>
                    </div>
                    <div class="modal-actions">
                        <button class="btn btn-secondary" onclick="closePauseDialog()">Cancel</button>
                    </div>
                </div>
            `;
            document.body.appendChild(modal);
            modal.addEventListener('click', function(e) {
                if (e.target === modal) closePauseDialog();
            });
        }

        function closePauseDialog() {
            const modal = document.getElementById('pauseModal');
            if (modal) modal.remove();
            currentPauseServiceId = null;
        }

        // Show alert
        function showAlert(message, type) {
            const container = document.getElementById('alertContainer');
            const alert = document.createElement('div');
            alert.className = `alert alert-${type}`;
            alert.textContent = message;
            container.appendChild(alert);

            setTimeout(() => {
                alert.remove();
            }, 3000);
        }

        // Export services
        function exportServices() {
            window.location.href = '/api/export';
        }

        // Import services
        async function importServices(file) {
            if (!file) return;

            try {
                const text = await file.text();
                const response = await fetch('/api/import', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: text
                });

                const result = await response.json();

                if (response.ok) {
                    showAlert(`Imported ${result.imported} service(s)` + 
                        (result.skipped > 0 ? `, skipped ${result.skipped}` : ''), 'success');
                    loadServices();
                } else {
                    showAlert('Import failed: ' + (result.error || 'Unknown error'), 'error');
                }
            } catch (error) {
                showAlert('Error: ' + error.message, 'error');
            }

            // Reset file input
            document.getElementById('importFile').value = '';
        }

        // Auto-refresh services every 5 seconds
        setInterval(loadServices, 5000);

        // Initial load
        loadServices();
        document.getElementById('serviceType').dispatchEvent(new Event('change'));
    </script>
</body>
</html>
)rawliteral";
}
