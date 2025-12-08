#pragma once

#include <cstdint>

// Wi-Fi credentials
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;

// Web authentication (Basic Auth)
extern const char* WEB_AUTH_USERNAME;
extern const char* WEB_AUTH_PASSWORD;

// BLE configuration (for boards without built-in LoRa radio)
extern const char* BLE_DEVICE_NAME;
extern const char* BLE_PEER_NAME;
extern const uint32_t BLE_PAIRING_PIN;
extern const char* BLE_MESH_CHANNEL_NAME;
extern const char* BLE_MESH_ROOM_SERVER_ID;
extern const char* BLE_MESH_ROOM_SERVER_PASSWORD;

// MeshCore LoRa group channel pre-shared key (Base64). Used when HAS_LORA_RADIO is defined.
// Default is the public channel PSK. Override via build flag MESH_LORA_CHANNEL_PSK_BASE64_VALUE.
extern const char* LORA_MESH_CHANNEL_PSK_BASE64;

// LoRa radio configuration (for boards with built-in SX1262 radio)
// These are used when HAS_LORA_RADIO is defined (e.g., Heltec Wireless Stick Lite V3)
extern const float LORA_FREQUENCY;          // MHz (default: 915.0 for US915)
extern const float LORA_BANDWIDTH;          // kHz (default: 250.0)
extern const uint8_t LORA_SPREADING_FACTOR; // 7-12 (default: 10)
extern const uint8_t LORA_CODING_RATE;      // 5-8 for 4/5 to 4/8 (default: 5)
extern const uint16_t LORA_SYNC_WORD;       // Sync word (MeshCore uses private 0x1424)
extern const int8_t LORA_TX_POWER;          // dBm (default: 22)
extern const int LORA_TX_LED_PIN;           // Optional GPIO to pulse during LoRa TX (-1 to disable)
extern const int LORA_VEXT_PIN;             // Optional GPIO to enable external power (e.g. 21 on Heltec V3)
extern const uint16_t LORA_PREAMBLE_LENGTH; // LoRa preamble symbols (default 16 to match MeshCore)
extern const float LORA_TCXO_VOLTAGE;       // TCXO voltage for SX126x (e.g., 1.6 for Heltec)

// ntfy configuration
extern const char* NTFY_SERVER;
extern const char* NTFY_TOPIC;
extern const char* NTFY_ACCESS_TOKEN;
extern const char* NTFY_USERNAME;
extern const char* NTFY_PASSWORD;

// Discord configuration
extern const char* DISCORD_WEBHOOK_URL;

// SMTP configuration
extern const char* SMTP_SERVER;
extern const int SMTP_PORT;
extern const bool SMTP_USE_TLS;
extern const char* SMTP_USERNAME;
extern const char* SMTP_PASSWORD;
extern const char* SMTP_FROM_ADDRESS;
extern const char* SMTP_TO_ADDRESS;

// Boot notification configuration
extern const bool BOOT_NOTIFICATION_ENABLED;

// RGB LED configuration
extern const bool LED_ENABLED;

// Screen timeout configuration (in seconds, 0 = disabled)
extern const int SCREEN_TIMEOUT;
