#include <Arduino.h>
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
#define TOUCH_INT_PIN 40  // Touch interrupt pin
#endif

#ifndef TOUCH_RST_PIN
// Touch reset pin - Hardware design note:
// GPIO 38 is shared between touch reset and backlight control on this board.
// The GT911 touch controller requires a reset pulse during initialization,
// after which the pin is reconfigured for PWM backlight control by LovyanGFX.
#define TOUCH_RST_PIN 38
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

    // Backlight (PWM)
    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = TFT_BL_PIN;
      _light_instance.config(cfg);
    }
    _panel_instance.light(&_light_instance);

    // GT911 Touch controller configuration
    {
      auto cfg = _touch_instance.config();
      cfg.x_min = 0;
      cfg.x_max = TFT_WIDTH - 1;
      cfg.y_min = 0;
      cfg.y_max = TFT_HEIGHT - 1;
      cfg.pin_int = TOUCH_INT_PIN;
      cfg.pin_rst = TOUCH_RST_PIN;
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;
      cfg.i2c_port = 1;
      cfg.i2c_addr = 0x5D;  // GT911 default address
      cfg.pin_sda = TOUCH_SDA_PIN;
      cfg.pin_scl = TOUCH_SCL_PIN;
      cfg.freq = 400000;
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
static int currentServiceIndex = 0;
static bool displayNeedsUpdate = true;
static unsigned long lastDisplaySwitch = 0;
static const unsigned long DISPLAY_ROTATION_INTERVAL = 8000;
static unsigned long lastTouchTime = 0;
static const unsigned long TOUCH_DEBOUNCE_MS = 300;  // Debounce interval for touch input

// Function prototypes for LCD
void initDisplay();
void renderServiceOnDisplay();
void handleDisplayLoop();
#endif // HAS_LCD

// RGB LED Status Indicator
// Uses the built-in RGB LED on ESP32-S3 DevKitC (GPIO 48)
// LED states indicate system and service health at a glance

enum LedStatus {
  LED_STATUS_BOOTING,      // Blue pulsing - system booting, no checks yet
  LED_STATUS_NO_WIFI,      // Orange - no WiFi connection
  LED_STATUS_MESHCORE,     // White - communicating with MeshCore radio
  LED_STATUS_ALL_UP,       // Green pulsing - all services are UP
  LED_STATUS_ANY_DOWN      // Red pulsing - one or more services are DOWN
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
                      currentLedStatus == LED_STATUS_ANY_DOWN);
  
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
  // MeshCore is configured if peer name is set AND (channel OR room server is set)
  return strlen(BLE_PEER_NAME) > 0 && 
         (strlen(BLE_MESH_CHANNEL_NAME) > 0 || strlen(BLE_MESH_ROOM_SERVER_ID) > 0);
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
// Layer 1: BLE Transport - handles BLE connection and I/O
// Layer 2: Frame Codec - handles frame parsing/building  
// Layer 3: Companion Protocol - handles MeshCore protocol logic
static BLECentralTransport* meshTransport = nullptr;
static FrameCodec* meshCodec = nullptr;
static CompanionProtocol* meshProtocol = nullptr;

// BLE/WiFi coexistence - ESP32-S3 cannot run WiFi and BLE simultaneously
bool bleOperationInProgress = false;
bool monitoringPaused = false;

// Minimum valid Unix timestamp for NTP validation (2021-01-01 00:00:00 UTC)
// Used to detect if time has been properly synchronized via NTP
const time_t MIN_VALID_TIMESTAMP = 1609459200;

// Pending MeshCore notification - used to defer BLE operations from HTTP handlers
// This prevents task watchdog timeouts by allowing the async web server to complete
// HTTP response delivery before WiFi is disconnected for BLE operations.
volatile bool pendingMeshNotification = false;
String pendingMeshTitle = "";
String pendingMeshMessage = "";

// Helper functions to access protocol state (for API compatibility)
bool isMeshDeviceConnected() {
  return meshTransport != nullptr && meshTransport->isConnected();
}

bool isMeshChannelReady() {
  return meshProtocol != nullptr && meshProtocol->isChannelReady();
}

String getMeshLastError() {
  if (meshProtocol != nullptr) {
    return meshProtocol->getLastError();
  }
  if (meshTransport != nullptr) {
    return meshTransport->getLastError();
  }
  return "";
}

int getMeshProtocolState() {
  if (meshProtocol != nullptr) {
    return static_cast<int>(meshProtocol->getState());
  }
  return 0;
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
  delay(1000);

  Serial.println("Starting ESP32 Uptime Monitor...");

  // Initialize LED to booting state (blue pulsing)
  setLedStatus(LED_STATUS_BOOTING);

  // Initialize filesystem
  initFileSystem();

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
    // Check if any enabled service is down
    bool anyDown = false;
    for (int i = 0; i < serviceCount; i++) {
      if (services[i].enabled && !services[i].isUp && services[i].lastCheck > 0) {
        anyDown = true;
        break;
      }
    }
    setLedStatus(anyDown ? LED_STATUS_ANY_DOWN : LED_STATUS_ALL_UP);
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
  // First attempt: try to mount without formatting
  if (LittleFS.begin(false)) {
    Serial.println("LittleFS mounted successfully");
    return;
  }

  Serial.println("LittleFS mount failed, attempting format...");

  // Second attempt: begin(true) formats automatically on mount failure and
  // handles partition initialization internally, which is more reliable than
  // calling format() directly on corrupted or uninitialized flash.
  if (LittleFS.begin(true)) {
    Serial.println("LittleFS formatted and mounted successfully");
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
    return;
  }

  Serial.println("LittleFS mounted successfully after format");
}

void disconnectMeshCore() {
  // Clear callbacks before cleanup to prevent use-after-free.
  // BLE callbacks could fire during disconnect/deinit and would reference deleted objects.
  if (meshCodec != nullptr) {
    meshCodec->clearCallbacks();
  }
  
  // Clean up the layered protocol stack
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
    if (!ensureAuthenticated(request)) {
      return;
    }
    request->send(200, "text/html", getWebPage());
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

      int failThreshold = doc["failThreshold"] | 1;
      if (failThreshold < 1) failThreshold = 1;
      newService.failThreshold = failThreshold;

      int rearmCount = doc["rearmCount"] | 0;
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
  server.on("^\\/api\\/services\\/([a-zA-Z0-9]+)$", HTTP_PATCH, [](AsyncWebServerRequest *request) {}, NULL,
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

        int failThreshold = obj["failThreshold"] | 1;
        if (failThreshold < 1) failThreshold = 1;

        int rearmCount = obj["rearmCount"] | 0;
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
        services[i].lastPush = millis();
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

#ifdef HAS_LCD
      displayNeedsUpdate = true;
#endif
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
    // Update display if the currently shown service was checked
    if (i == currentServiceIndex) {
      displayNeedsUpdate = true;
    }
#endif
  }
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
  
  if (isSecure) {
    WiFiClientSecure secureClient;
    secureClient.setInsecure();  // Skip certificate validation
    http.begin(secureClient, url);
  } else {
    http.begin(url);
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
  // ESP32-S3 cannot run WiFi and BLE simultaneously
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
    sendMeshCoreNotification(title, message);
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
  // ESP32-S3 cannot run WiFi and BLE simultaneously
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
    services[serviceCount].failThreshold = obj["failThreshold"] | 1;
    services[serviceCount].rearmCount = obj["rearmCount"] | 0;
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

void initDisplay() {
  Serial.println("Initializing display...");
  
  // Reset the GT911 touch controller before display initialization.
  // Hardware note: On ESP32-4848S040, GPIO 38 is shared between touch reset and backlight.
  // This reset sequence is required for proper GT911 I2C communication.
  if (TOUCH_RST_PIN >= 0) {
    pinMode(TOUCH_RST_PIN, OUTPUT);
    digitalWrite(TOUCH_RST_PIN, LOW);
    delay(10);  // GT911 requires minimum 10ms reset pulse
    digitalWrite(TOUCH_RST_PIN, HIGH);
    delay(50);  // Wait for GT911 to complete internal initialization
    Serial.println("GT911 touch controller reset complete");
  }
  
  displayReady = display.init();
  display.setRotation(0);  // Rotation 0 for ESP32-4848S040 square display
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);

  if (TFT_BL_PIN >= 0) {
    display.setBrightness(200);
  }

  // Touch controller is initialized as part of the LGFX class (GT911)
  touchReady = display.touch() != nullptr;

  if (touchReady) {
    Serial.println("Touch controller (GT911) ready");
  } else {
    Serial.println("Touch controller not detected");
  }

  if (displayReady) {
    display.fillScreen(TFT_BLACK);
    renderServiceOnDisplay();
    lastDisplaySwitch = millis();
    Serial.println("Display initialized successfully");
  } else {
    Serial.println("Display initialization failed");
  }
}

void renderServiceOnDisplay() {
  if (!displayReady) return;

  display.fillScreen(TFT_BLACK);

  int16_t width = display.width();
  int16_t height = display.height();

  // Header with IP address
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.setCursor(10, 10);
  display.setTextSize(2);
  if (WiFi.status() == WL_CONNECTED) {
    String header = "ESP32 Monitor - " + WiFi.localIP().toString();
    display.println(header);
  } else {
    display.println("ESP32 Monitor - No WiFi");
  }

  // Show message if no services configured
  if (serviceCount == 0) {
    display.setCursor(10, 60);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.println("No services configured.");
    display.println("Add services via web UI.");
    return;
  }

  // Ensure index is valid
  if (currentServiceIndex >= serviceCount) {
    currentServiceIndex = 0;
  }

  Service& svc = services[currentServiceIndex];
  
  // Determine status text and color based on check state
  String status;
  uint16_t statusColor;
  if (svc.lastCheck == 0) {
    // Service hasn't been checked yet
    status = "PENDING";
    statusColor = TFT_BLUE;
  } else if (svc.isUp) {
    status = "UP";
    statusColor = TFT_GREEN;
  } else {
    status = "DOWN";
    statusColor = TFT_RED;
  }

  // Service name with pagination
  display.setTextSize(3);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(10, 50);
  display.printf("%s (%d/%d)", svc.name.c_str(), currentServiceIndex + 1, serviceCount);

  // Status box
  display.fillRoundRect(10, 90, width - 20, 60, 12, TFT_NAVY);
  display.setTextSize(2);
  display.setCursor(20, 110);
  display.setTextColor(statusColor, TFT_NAVY);
  display.printf("Status: %s", status.c_str());

  // Service type
  display.setCursor(20, 150);
  display.setTextColor(TFT_YELLOW, TFT_BLACK);
  display.printf("Type: %s", getServiceTypeString(svc.type).c_str());

  // Host information (not applicable for PUSH type)
  if (svc.type != TYPE_PUSH) {
    display.setCursor(20, 180);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    if (svc.type == TYPE_HTTP_GET && svc.url.length() > 0) {
      // For HTTP GET, show URL (truncated if too long)
      String urlDisplay = svc.url;
      if (urlDisplay.length() > 30) {
        urlDisplay = urlDisplay.substring(0, 27) + "...";
      }
      display.printf("URL: %s", urlDisplay.c_str());
    } else if (svc.type == TYPE_PING) {
      display.printf("Host: %s", svc.host.c_str());
    } else {
      display.printf("Host: %s:%d", svc.host.c_str(), svc.port);
    }
  }

  // Last check timestamp
  unsigned long sinceCheck = svc.lastCheck > 0 ? (millis() - svc.lastCheck) / 1000 : 0;
  display.setCursor(20, 210);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  if (svc.lastCheck == 0) {
    display.println("Last check: pending");
  } else {
    display.printf("Last check: %lus ago", sinceCheck);
  }

  // Show enabled/paused status
  display.setCursor(20, 240);
  if (!svc.enabled) {
    display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    display.println("Status: Disabled");
  } else if (svc.pauseUntil > 0) {
    unsigned long remaining = getPauseRemainingMs(svc.pauseUntil, millis());
    if (remaining > 0) {
      display.setTextColor(TFT_ORANGE, TFT_BLACK);
      display.printf("Paused: %lus remaining", remaining / 1000);
    }
  }

  // Error message if present
  if (svc.lastError.length() > 0) {
    display.setCursor(20, 270);
    display.setTextColor(TFT_RED, TFT_BLACK);
    // Truncate error message if too long
    String errorDisplay = svc.lastError;
    if (errorDisplay.length() > 35) {
      errorDisplay = errorDisplay.substring(0, 32) + "...";
    }
    display.printf("Error: %s", errorDisplay.c_str());
  }

  // Navigation instructions at bottom
  display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  display.setCursor(10, height - 60);
  display.println("Tap left/right to switch");
  display.setCursor(10, height - 30);
  display.printf("Auto-rotate every %lus", DISPLAY_ROTATION_INTERVAL / 1000);
}

void handleDisplayLoop() {
  if (!displayReady) return;

  unsigned long now = millis();

  // Auto-rotate through services
  if (serviceCount > 0 && now - lastDisplaySwitch >= DISPLAY_ROTATION_INTERVAL) {
    currentServiceIndex = (currentServiceIndex + 1) % serviceCount;
    displayNeedsUpdate = true;
    lastDisplaySwitch = now;
  }

  // Handle touch input for manual navigation (non-blocking debounce)
  if (touchReady && serviceCount > 0 && (now - lastTouchTime >= TOUCH_DEBOUNCE_MS)) {
    lgfx::touch_point_t tp;
    int touchCount = display.getTouch(&tp, 1);
    
    if (touchCount > 0) {
      int16_t x = tp.x;
      int16_t y = tp.y;

      // Only respond to touches below the header area
      if (y > 20) {
        if (x < display.width() / 2) {
          // Left side - previous service
          currentServiceIndex = (currentServiceIndex - 1 + serviceCount) % serviceCount;
        } else {
          // Right side - next service
          currentServiceIndex = (currentServiceIndex + 1) % serviceCount;
        }
        displayNeedsUpdate = true;
        lastDisplaySwitch = now;
        lastTouchTime = now;  // Record touch time for debounce
      }
    }
  }

  // Update display if needed
  if (displayNeedsUpdate) {
    renderServiceOnDisplay();
    displayNeedsUpdate = false;
  }
}

#endif // HAS_LCD

String getWebPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Uptime Monitor</title>
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
            <h1>ESP32 Uptime Monitor</h1>
            <p>Monitor your services and infrastructure health</p>
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
                        <input type="number" id="failThreshold" value="1" required min="1" title="Number of consecutive failures before marking as DOWN">
                    </div>

                    <div class="form-group">
                        <label for="passThreshold">Pass Threshold</label>
                        <input type="number" id="passThreshold" value="1" required min="1" title="Number of consecutive successes before marking as UP">
                    </div>
                </div>

                <div class="form-group">
                    <label for="rearmCount">Re-arm Alert Count (0 = disabled)</label>
                    <input type="number" id="rearmCount" value="0" required min="0" title="Number of failed checks before re-alerting while service is DOWN. Set to 0 to disable.">
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

        <h2 style="color: white; margin-bottom: 20px; font-size: 1.5em;">Monitored Services</h2>
        <div id="servicesContainer" class="services-grid"></div>
        <div id="emptyState" class="empty-state hidden">
            <h3>No services yet</h3>
            <p>Add your first service using the form above</p>
        </div>
    </div>

    <script>
        let services = [];

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

            try {
                const response = await fetch('/api/services', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });

                if (response.ok) {
                    showAlert('Service added successfully!', 'success');
                    this.reset();
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
            const container = document.getElementById('servicesContainer');
            const emptyState = document.getElementById('emptyState');

            if (services.length === 0) {
                container.innerHTML = '';
                emptyState.classList.remove('hidden');
                return;
            }

            emptyState.classList.add('hidden');

            container.innerHTML = services.map(service => {
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

                const rearmInfo = service.rearmCount > 0 
                    ? `${service.rearmCount} (${service.failedChecksSinceAlert} since last alert)` 
                    : 'disabled';

                // Build SNMP info section if applicable
                let snmpInfo = '';
                if (service.type === 'snmp_get' && service.snmpOid) {
                    snmpInfo = `
                        <div class="service-info">
                            <strong>OID:</strong> ${service.snmpOid}
                        </div>
                        <div class="service-info">
                            <strong>Community:</strong> ${service.snmpCommunity}
                        </div>
                        <div class="service-info">
                            <strong>Check:</strong> value ${service.snmpCompareOp} ${service.snmpExpectedValue}
                        </div>
                    `;
                }

                // Build push info section if applicable
                let pushInfo = '';
                if (service.type === 'push' && service.pushToken) {
                    const pushUrl = window.location.origin + '/api/push/' + service.pushToken;
                    pushInfo = `
                        <div class="service-info">
                            <strong>Push URL:</strong> <code style="font-size: 0.85em; background: #f3f4f6; padding: 2px 6px; border-radius: 4px; word-break: break-all;">${pushUrl}</code>
                        </div>
                    `;
                }

                // Build URL info for http_get type
                let urlInfo = '';
                if (service.type === 'http_get' && service.url) {
                    urlInfo = `
                        <div class="service-info">
                            <strong>URL:</strong> <span style="word-break: break-all;">${service.url}</span>
                        </div>
                    `;
                }

                // Build host info - not applicable for push or http_get type
                let hostInfo = '';
                if (service.type !== 'push' && service.type !== 'http_get') {
                    hostInfo = `
                        <div class="service-info">
                            <strong>Host:</strong> ${service.host}${service.type !== 'ping' ? ':' + service.port : ''}
                        </div>
                    `;
                }

                // Build status info for enabled/paused state
                let statusInfo = '';
                if (!service.enabled) {
                    statusInfo = '<div class="service-info" style="color: #6b7280;"><strong>Status:</strong> Disabled</div>';
                } else if (service.pauseRemaining > 0) {
                    const pauseMins = Math.floor(service.pauseRemaining / 60);
                    const pauseSecs = service.pauseRemaining % 60;
                    const pauseStr = pauseMins > 0 ? `${pauseMins}m ${pauseSecs}s` : `${pauseSecs}s`;
                    statusInfo = `<div class="service-info" style="color: #f59e0b;"><strong>Status:</strong> Paused (${pauseStr} remaining)</div>`;
                }

                // Determine if service is pending (never checked)
                const isPending = service.secondsSinceLastCheck < 0;

                // Determine service card class based on enabled/paused/pending state
                let cardClass = service.isUp ? 'up' : 'down';
                if (isPending) {
                    cardClass = 'pending';
                }
                if (!service.enabled || service.pauseRemaining > 0) {
                    cardClass = 'paused';
                }

                // Determine status text and class
                let statusText = service.isUp ? 'UP' : 'DOWN';
                let statusClass = service.isUp ? 'up' : 'down';
                if (isPending) {
                    statusText = 'PENDING';
                    statusClass = 'pending';
                }

                return `
                    <div class="service-card ${cardClass}">
                        <div class="service-header">
                            <div>
                                <div class="service-name">${service.name}</div>
                                <div class="type-badge">${service.type.replace('_', ' ').toUpperCase()}</div>
                            </div>
                            <span class="service-status ${statusClass}">
                                ${statusText}
                            </span>
                        </div>
                        ${statusInfo}
                        ${urlInfo}
                        ${hostInfo}
                        ${snmpInfo}
                        ${pushInfo}
                        <div class="service-info">
                            <strong>Check Interval:</strong> ${service.checkInterval}s
                        </div>
                        <div class="service-info">
                            <strong>Thresholds:</strong> ${service.failThreshold} fail / ${service.passThreshold} pass
                        </div>
                        <div class="service-info">
                            <strong>Re-arm Alert:</strong> ${rearmInfo}
                        </div>
                        <div class="service-info">
                            <strong>Consecutive:</strong> ${service.consecutivePasses} passes / ${service.consecutiveFails} fails
                        </div>
                        <div class="service-info">
                            <strong>Last Check:</strong> ${uptimeStr}
                        </div>
                        ${service.lastError ? `
                        <div class="service-info" style="color: #ef4444;">
                            <strong>Error:</strong> ${service.lastError}
                        </div>
                        ` : ''}
                        <div class="service-actions">
                            ${service.enabled 
                                ? `<button class="btn btn-secondary" onclick="toggleService('${service.id}', false)">Disable</button>`
                                : `<button class="btn btn-primary" onclick="toggleService('${service.id}', true)">Enable</button>`
                            }
                            ${service.pauseRemaining > 0
                                ? `<button class="btn btn-secondary" onclick="pauseService('${service.id}', 0)">Unpause</button>`
                                : `<button class="btn btn-secondary" onclick="showPauseDialog('${service.id}')">Pause</button>`
                            }
                            <button class="btn btn-danger" onclick="deleteService('${service.id}')">Delete</button>
                        </div>
                    </div>
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
