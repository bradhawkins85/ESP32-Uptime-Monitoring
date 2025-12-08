#pragma once

#include "IByteTransport.hpp"
#include <Arduino.h>
#include <SPI.h>

// Only compile LoRa transport when HAS_LORA_RADIO is defined
#ifdef HAS_LORA_RADIO
#include <RadioLib.h>

/**
 * LoRaTransport - LoRa-based transport layer using SX1262 radio
 * 
 * This class handles:
 * - SX1262 radio initialization and configuration
 * - MeshCore packet transmission over LoRa
 * - Receiving packets from the mesh network
 * 
 * Implements IByteTransport interface for use by higher protocol layers.
 * 
 * Note: Unlike BLE transport which connects to a specific peer device,
 * LoRa transport broadcasts to all nodes in range on the configured
 * frequency and spreading factor (compatible with MeshCore network).
 */
class LoRaTransport : public IByteTransport {
public:
    // MeshCore LoRa default parameters (must match MeshCore network settings)
    static constexpr float DEFAULT_FREQUENCY = 915.0;     // MHz (US915 band)
    static constexpr float DEFAULT_BANDWIDTH = 250.0;     // kHz
    static constexpr uint8_t DEFAULT_SPREADING_FACTOR = 10;
    static constexpr uint8_t DEFAULT_CODING_RATE = 5;     // 4/5
    // MeshCore uses the LoRa private sync word (0x1424) on SX126x
    static constexpr uint16_t DEFAULT_SYNC_WORD = 0x1424;
    static constexpr int8_t DEFAULT_TX_POWER = 22;        // dBm (max for SX1262)
    // MeshCore radios typically use a 16-symbol preamble; increase default to match
    static constexpr uint16_t DEFAULT_PREAMBLE_LENGTH = 16;
    static constexpr float DEFAULT_TCXO_VOLTAGE = -1.0f;  // -1 = no TCXO; 1.6f common on Heltec
    
    // Maximum packet size for LoRa transmissions
    static constexpr size_t MAX_PACKET_SIZE = 255;

    // Configuration for LoRa radio
    struct Config {
        // SPI pins for SX1262
        int8_t pinNss = 8;
        int8_t pinDio1 = 14;
        int8_t pinRst = 12;
        int8_t pinBusy = 13;
        int8_t pinMosi = 10;
        int8_t pinMiso = 11;
        int8_t pinSck = 9;
        
        // Radio parameters
        float frequency = DEFAULT_FREQUENCY;
        float bandwidth = DEFAULT_BANDWIDTH;
        uint8_t spreadingFactor = DEFAULT_SPREADING_FACTOR;
        uint8_t codingRate = DEFAULT_CODING_RATE;
        uint16_t syncWord = DEFAULT_SYNC_WORD;
        int8_t txPower = DEFAULT_TX_POWER;
        uint16_t preambleLength = DEFAULT_PREAMBLE_LENGTH;
        int8_t txLedPin = -1;            // Optional GPIO to pulse during TX (-1 disables)
        int8_t pinVext = -1;             // Optional GPIO to enable external power (-1 disables)
        float tcxoVoltage = DEFAULT_TCXO_VOLTAGE; // Set >0 to enable TCXO on DIO3
        
        // Transmission retry parameters
        uint8_t maxTransmitRetries = 3;  // Number of times to retry transmission on failure
        uint8_t maxCadRetries = 5;       // Number of times to retry CAD when channel is busy
        uint16_t cadRetryDelayMs = 500;  // Delay between CAD retries when channel is busy
        uint16_t txRetryDelayMs = 1000;  // Delay between transmission retries
    };

    explicit LoRaTransport(const Config& config);
    ~LoRaTransport() override;

    // IByteTransport interface
    bool send(const uint8_t* data, size_t len) override;
    bool isConnected() const override;
    void setRxCallback(RxCallback callback) override;
    void setStateCallback(StateCallback callback) override;
    void clearCallbacks() override;

    // LoRa-specific methods
    
    /**
     * Initialize LoRa radio
     * @return true if initialization succeeded
     */
    bool init();

    /**
     * Deinitialize LoRa radio
     */
    void deinit();

    /**
     * Check if LoRa is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * Process any received packets (should be called in loop)
     * This checks for incoming packets and dispatches to callback
     */
    void processReceive();

    /**
     * Get last error message
     * @return Last error message string
     */
    const String& getLastError() const { return m_lastError; }

    /**
     * Set to receive mode after transmission
     */
    void startReceive();

private:
    Config m_config;
    bool m_initialized = false;
    
    // RadioLib module and radio instances
    // Using pointers to allow deferred initialization
    SPIClass* m_spi = nullptr;
    Module* m_module = nullptr;
    SX1262* m_radio = nullptr;
    
    RxCallback m_rxCallback;
    StateCallback m_stateCallback;
    
    String m_lastError;
    
    // Receive buffer
    uint8_t m_rxBuffer[MAX_PACKET_SIZE];
};

#endif // HAS_LORA_RADIO
