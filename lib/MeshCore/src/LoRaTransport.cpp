#include "LoRaTransport.hpp"

#ifdef HAS_LORA_RADIO

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define static constexpr members for pre-C++17 ODR compliance
constexpr float LoRaTransport::DEFAULT_FREQUENCY;
constexpr float LoRaTransport::DEFAULT_BANDWIDTH;
constexpr uint8_t LoRaTransport::DEFAULT_SPREADING_FACTOR;
constexpr uint8_t LoRaTransport::DEFAULT_CODING_RATE;
constexpr uint16_t LoRaTransport::DEFAULT_SYNC_WORD;
constexpr int8_t LoRaTransport::DEFAULT_TX_POWER;
constexpr uint16_t LoRaTransport::DEFAULT_PREAMBLE_LENGTH;
constexpr size_t LoRaTransport::MAX_PACKET_SIZE;

LoRaTransport::LoRaTransport(const Config& config)
    : m_config(config)
{
}

LoRaTransport::~LoRaTransport() {
    deinit();
}

bool LoRaTransport::init() {
    Serial.println("LoRaTransport: Initializing SX1262 radio...");
    Serial.printf("Free heap before LoRa init: %d bytes\n", ESP.getFreeHeap());
    
    // Initialize SPI for the radio
    m_spi = new SPIClass(HSPI);
    m_spi->begin(m_config.pinSck, m_config.pinMiso, m_config.pinMosi, m_config.pinNss);

    // Enable Vext if configured (common on Heltec boards to power sensors/RF switch)
    if (m_config.pinVext >= 0) {
        pinMode(m_config.pinVext, OUTPUT);
        digitalWrite(m_config.pinVext, LOW); // Active LOW for Heltec V3 Vext
        delay(50); // Allow power to stabilize
        Serial.printf("LoRaTransport: Vext enabled on pin %d\n", m_config.pinVext);
    } else {
        Serial.println("LoRaTransport: Vext not configured (pinVext = -1)");
    }

    // Configure TX LED pin if provided
    if (m_config.txLedPin >= 0) {
        pinMode(m_config.txLedPin, OUTPUT);
        digitalWrite(m_config.txLedPin, LOW);
    }
    
    // Create RadioLib module with pin configuration
    m_module = new Module(m_config.pinNss, m_config.pinDio1, m_config.pinRst, m_config.pinBusy, *m_spi);
    
    // Create SX1262 radio instance
    m_radio = new SX1262(m_module);
    
    // Initialize the radio with MeshCore-compatible parameters
    // RadioLib SX1262 uses a 1-byte sync word; mask to 8 bits and warn if caller provided more
    uint8_t syncWordByte = static_cast<uint8_t>(m_config.syncWord & 0xFF);
    if (m_config.syncWord > 0xFF) {
        Serial.printf("Warning: sync word 0x%04X truncated to 0x%02X (SX1262 uses 1 byte)\n",
                      m_config.syncWord, syncWordByte);
    }

    Serial.printf("LoRa config: %.3f MHz, BW %.1f kHz, SF %d, CR 4/%d, SW 0x%02X, PRE=%u, TCXO=%.2f\n",
                  m_config.frequency, m_config.bandwidth,
                  m_config.spreadingFactor, m_config.codingRate,
                  syncWordByte, m_config.preambleLength,
                  m_config.tcxoVoltage);

    int state = m_radio->begin(
        m_config.frequency,
        m_config.bandwidth,
        m_config.spreadingFactor,
        m_config.codingRate,
        syncWordByte,
        m_config.txPower,
        m_config.preambleLength,
        m_config.tcxoVoltage
    );
    
    if (state != RADIOLIB_ERR_NONE) {
        m_lastError = "SX1262 init failed with code: " + String(state);
        Serial.println(m_lastError);
        deinit();
        return false;
    }
    
    // Configure for explicit header mode (used by MeshCore)
    state = m_radio->explicitHeader();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: explicit header mode failed: %d\n", state);
    }
    
    // Set CRC (can be disabled via build flag for interop testing)
    bool crcOn = true;
#ifdef LORA_DISABLE_CRC
    crcOn = false;
#endif
    state = m_radio->setCRC(crcOn);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: setCRC(%d) failed: %d\n", crcOn ? 1 : 0, state);
    } else {
        Serial.printf("LoRa CRC %s\n", crcOn ? "ENABLED" : "DISABLED");
    }

    // Allow IQ inversion to be toggled via build flag for interoperability testing
    bool invertIq = false;
#ifdef LORA_INVERT_IQ
    invertIq = true;
#endif
    state = m_radio->invertIQ(invertIq);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: invertIQ(%d) failed: %d\n", invertIq ? 1 : 0, state);
    } else {
        Serial.printf("LoRa IQ %s\n", invertIq ? "INVERTED" : "NORMAL");
    }
    
    // Use DIO2 as RF switch control (common on Heltec boards)
    state = m_radio->setDio2AsRfSwitch(true);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: DIO2 RF switch config failed: %d\n", state);
    }
    
    // Explicitly set output power (ensure it's actually applied)
    // RadioLib handles PA config automatically when setOutputPower is called
    state = m_radio->setOutputPower(m_config.txPower);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: setOutputPower(%d) failed: %d\n", m_config.txPower, state);
    } else {
        Serial.printf("Output power explicitly set to %d dBm\n", m_config.txPower);
    }
    
    // Enable RX boosted gain for better sensitivity (SX1262 specific)
    if (m_radio->setRxBoostedGainMode(true) != RADIOLIB_ERR_NONE) {
        Serial.println("Warning: Failed to set RX boosted gain");
    } else {
        Serial.println("RX boosted gain enabled");
    }

    // Set Over Current Protection to 140mA (required for +22dBm)
    state = m_radio->setCurrentLimit(140);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: setCurrentLimit(140) failed: %d\n", state);
    } else {
        Serial.println("Current limit set to 140mA");
    }
    
    m_initialized = true;
    m_lastError = "";
    
    Serial.println("SX1262 radio initialized successfully");
    Serial.printf("Free heap after LoRa init: %d bytes\n", ESP.getFreeHeap());
    
    // Notify state callback that we're "connected" (ready to send)
    if (m_stateCallback) {
        m_stateCallback(true);
    }
    
    // CRITICAL: Force radio into RX mode and verify (no verbose OK prints to avoid log spam)
    
    // Ensure radio is in standby first
    state = m_radio->standby();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: standby() failed: %d\n", state);
    }
    
    // Small delay to ensure state transition completes
    delay(10);
    
    // Start listening for incoming packets
    int rxState = m_radio->startReceive();
    if (rxState != RADIOLIB_ERR_NONE) {
        Serial.printf("ERROR: startReceive failed at init: %d\n", rxState);
        Serial.println("[LoRa] RX MODE FAILED - Radio cannot receive packets!");
    }
    
    return true;
}

void LoRaTransport::deinit() {
    if (m_radio != nullptr) {
        m_radio->standby();
        delete m_radio;
        m_radio = nullptr;
    }
    
    if (m_module != nullptr) {
        delete m_module;
        m_module = nullptr;
    }
    
    if (m_spi != nullptr) {
        m_spi->end();
        delete m_spi;
        m_spi = nullptr;
    }
    
    if (m_initialized && m_stateCallback) {
        m_stateCallback(false);
    }
    
    m_initialized = false;
    Serial.println("LoRa radio deinitialized");
}

bool LoRaTransport::send(const uint8_t* data, size_t len) {
    if (!m_initialized || m_radio == nullptr) {
        m_lastError = "LoRa not initialized";
        return false;
    }
    
    if (len == 0 || len > MAX_PACKET_SIZE) {
        m_lastError = "Invalid packet size";
        return false;
    }

    Serial.printf("LoRaTransport TX: %d bytes\n", (int)len);
    
    // Verify radio configuration before sending
    Serial.printf("[TX] Pre-send: freq=%.3f MHz, BW=%.1f kHz, SF=%d, CR=4/%d, power=%d dBm\n",
                  m_config.frequency, m_config.bandwidth, 
                  m_config.spreadingFactor, m_config.codingRate, m_config.txPower);
    
    // Retry transmission loop
    for (uint8_t txAttempt = 0; txAttempt < m_config.maxTransmitRetries; txAttempt++) {
        if (txAttempt > 0) {
            Serial.printf("[TX] Retry attempt %d/%d after %d ms delay\n", 
                          txAttempt + 1, m_config.maxTransmitRetries, m_config.txRetryDelayMs);
            vTaskDelay(pdMS_TO_TICKS(m_config.txRetryDelayMs));
        }
        
        // Random delay (50-200ms) to reduce collision probability
        // Increased to 200-500ms to allow repeaters time to process and forward
        uint32_t randomDelay = 200 + (esp_random() % 301);
        Serial.printf("[TX] Random pre-tx delay: %lu ms\n", randomDelay);
        vTaskDelay(pdMS_TO_TICKS(randomDelay));
        
        // Channel Activity Detection with retries
        bool channelClear = false;
        for (uint8_t cadAttempt = 0; cadAttempt < m_config.maxCadRetries; cadAttempt++) {
            if (cadAttempt > 0) {
                Serial.printf("[TX] CAD retry %d/%d after %d ms\n", 
                              cadAttempt + 1, m_config.maxCadRetries, m_config.cadRetryDelayMs);
                vTaskDelay(pdMS_TO_TICKS(m_config.cadRetryDelayMs));
            }
            
            Serial.println("[TX] Performing CAD (Channel Activity Detection)...");
            int cadState = m_radio->scanChannel();
            
            if (cadState == RADIOLIB_CHANNEL_FREE) {
                Serial.println("[TX] Channel clear");
                channelClear = true;
                break;
            } else if (cadState == RADIOLIB_PREAMBLE_DETECTED) {
                Serial.printf("[TX] Channel busy (preamble detected) on CAD attempt %d/%d\n", 
                              cadAttempt + 1, m_config.maxCadRetries);
                // Continue to next CAD attempt
            } else {
                Serial.printf("[TX] CAD failed with error: %d\n", cadState);
                // On CAD error, assume channel is clear and proceed
                channelClear = true;
                break;
            }
        }
        
        if (!channelClear) {
            Serial.printf("[TX] Channel still busy after %d CAD attempts, aborting this transmission attempt\n", 
                          m_config.maxCadRetries);
            m_lastError = "Channel busy (collision avoidance)";
            startReceive();
            // Continue to next transmission retry
            continue;
        }
        
        // Indicate TX start if LED configured
        if (m_config.txLedPin >= 0) {
            digitalWrite(m_config.txLedPin, HIGH);
        }

        // Put radio in standby before transmitting
        m_radio->standby();
        
        // Transmit the packet (blocking)
        Serial.printf("[TX] Calling radio->transmit() (attempt %d/%d)...\n", 
                      txAttempt + 1, m_config.maxTransmitRetries);
        unsigned long txStart = millis();
        int state = m_radio->transmit(const_cast<uint8_t*>(data), len);
        unsigned long txDuration = millis() - txStart;
        
        if (state != RADIOLIB_ERR_NONE) {
            m_lastError = "Transmit failed with code: " + String(state);
            Serial.printf("[TX] FAILED: %s (took %lu ms, attempt %d/%d)\n", 
                          m_lastError.c_str(), txDuration, txAttempt + 1, m_config.maxTransmitRetries);
            // Turn off TX LED
            if (m_config.txLedPin >= 0) {
                digitalWrite(m_config.txLedPin, LOW);
            }
            // Return to receive mode
            startReceive();
            // Continue to next transmission retry
            continue;
        }
        
        // SUCCESS!
        Serial.printf("[TX] SUCCESS (took %lu ms, attempt %d/%d)\n", 
                      txDuration, txAttempt + 1, m_config.maxTransmitRetries);
        Serial.printf("[TX] Airtime estimate: ~%.1f ms (SF%d, BW%.1f, %d bytes)\n",
                      (float)len * 8.0f * (1 << m_config.spreadingFactor) / m_config.bandwidth,
                      m_config.spreadingFactor, m_config.bandwidth, (int)len);
        
        // Yield after transmission
        vTaskDelay(pdMS_TO_TICKS(5));
        
        // Return to receive mode after transmission
        startReceive();

        // Turn off TX LED
        if (m_config.txLedPin >= 0) {
            digitalWrite(m_config.txLedPin, LOW);
        }

        m_lastError = "";
        return true;
    }
    
    // All retries exhausted
    Serial.printf("[TX] All %d transmission attempts failed\n", m_config.maxTransmitRetries);
    m_lastError = "Transmission failed after " + String(m_config.maxTransmitRetries) + " retries";
    return false;
}

bool LoRaTransport::isConnected() const {
    // For LoRa, "connected" means the radio is initialized and ready
    // Unlike BLE, there's no persistent connection to a specific peer
    return m_initialized;
}

void LoRaTransport::setRxCallback(RxCallback callback) {
    m_rxCallback = std::move(callback);
}

void LoRaTransport::setStateCallback(StateCallback callback) {
    m_stateCallback = std::move(callback);
}

void LoRaTransport::clearCallbacks() {
    m_rxCallback = nullptr;
    m_stateCallback = nullptr;
}

void LoRaTransport::startReceive() {
    if (!m_initialized || m_radio == nullptr) {
        return;
    }

    int state = m_radio->startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("LoRaTransport: startReceive failed: %d\n", state);
    }
}

void LoRaTransport::processReceive() {
    if (!m_initialized || m_radio == nullptr) {
        return;
    }
    
    // Periodic diagnostic every 5 seconds
    static unsigned long lastDiag = 0;
    unsigned long now = millis();
    if (now - lastDiag >= 5000) {
        lastDiag = now;
        // getRSSI(false) returns instantaneous RSSI (noise floor) on SX126x
        // getRSSI(true) or default returns the RSSI of the LAST received packet
        Serial.printf("[RX] Polling... Instant RSSI: %.1f dBm\n", m_radio->getRSSI(false));
    }
    
    // Check if a packet was received by attempting to read
    // In polling mode (no interrupts), readData() returns:
    // - RADIOLIB_ERR_NONE: packet received successfully
    // - RADIOLIB_ERR_RX_TIMEOUT: no packet available (normal in polling)
    // - Other values: actual errors
    //
    // Note: For more efficient operation, interrupt mode could be used
    // with the DIO1 pin to signal packet arrival instead of polling.
    
    int state = m_radio->readData(m_rxBuffer, MAX_PACKET_SIZE);
    
    if (state == RADIOLIB_ERR_NONE) {
        // Get actual packet length after successful read
        size_t len = m_radio->getPacketLength();
        if (len > 0 && len <= MAX_PACKET_SIZE) {
            Serial.printf("LoRaTransport RX: %d bytes, RSSI: %.1f dBm, SNR: %.1f dB\n",
                          (int)len, m_radio->getRSSI(), m_radio->getSNR());
            
            // Forward to callback
            RxCallback callback = m_rxCallback;
            if (callback) {
                callback(m_rxBuffer, len);
            }
        }
        
        // Restart receive mode after successful read
        startReceive();
    } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
        // Actual error occurred (not just timeout)
        Serial.printf("LoRaTransport: read failed: %d\n", state);
        if (state == RADIOLIB_ERR_CRC_MISMATCH) {
             Serial.println("  (CRC Mismatch - signal detected but corrupted)");
        }
        
        // Restart receive mode after error
        startReceive();
    }
    // For RADIOLIB_ERR_RX_TIMEOUT, no action needed - still in receive mode
}

#endif // HAS_LORA_RADIO
