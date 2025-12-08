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
    Serial.printf("LoRa config: %.3f MHz, BW %.1f kHz, SF %d, CR 4/%d, SW 0x%04X, PRE=%u, TCXO=%.2f\n",
                  m_config.frequency, m_config.bandwidth,
                  m_config.spreadingFactor, m_config.codingRate,
                  m_config.syncWord, m_config.preambleLength,
                  m_config.tcxoVoltage);

    // Configure TCXO if provided (common on Heltec SX1262 boards)
    if (m_config.tcxoVoltage > 0) {
        int tcxoState = m_radio->setTCXO(m_config.tcxoVoltage);
        if (tcxoState != RADIOLIB_ERR_NONE) {
            Serial.printf("Warning: setTCXO(%.2f) failed: %d\n", m_config.tcxoVoltage, tcxoState);
        }
    }
    
    int state = m_radio->begin(
        m_config.frequency,
        m_config.bandwidth,
        m_config.spreadingFactor,
        m_config.codingRate,
        m_config.syncWord,
        m_config.txPower,
        m_config.preambleLength
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
    
    // Set CRC on for data integrity
    state = m_radio->setCRC(true);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: CRC enable failed: %d\n", state);
    }
    
    // Use DIO2 as RF switch control (common on Heltec boards)
    state = m_radio->setDio2AsRfSwitch(true);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: DIO2 RF switch config failed: %d\n", state);
    }
    
    // Explicitly set output power (ensure it's actually applied)
    state = m_radio->setOutputPower(m_config.txPower);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: setOutputPower(%d) failed: %d\n", m_config.txPower, state);
    } else {
        Serial.printf("Output power explicitly set to %d dBm\n", m_config.txPower);
    }
    
    // Configure PA (Power Amplifier) for SX1262 high power mode
    // Parameters: paDutyCycle, hpMax, deviceSel (SX1262), paLut (PA optimal)
    state = m_radio->setPaConfig(0x04, 0x07, 0x00, 0x01);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Warning: setPaConfig failed: %d\n", state);
    } else {
        Serial.println("PA configured for SX1262 (high power, +22 dBm)");
    }
    
    m_initialized = true;
    m_lastError = "";
    
    Serial.println("SX1262 radio initialized successfully");
    Serial.printf("Free heap after LoRa init: %d bytes\n", ESP.getFreeHeap());
    
    // Notify state callback that we're "connected" (ready to send)
    if (m_stateCallback) {
        m_stateCallback(true);
    }
    
    // Start listening for incoming packets
    int rxState = m_radio->startReceive();
    if (rxState != RADIOLIB_ERR_NONE) {
        Serial.printf("LoRaTransport: startReceive failed at init: %d\n", rxState);
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
    
    // Yield before transmission to feed watchdog
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Indicate TX start if LED configured
    if (m_config.txLedPin >= 0) {
        digitalWrite(m_config.txLedPin, HIGH);
    }

    // Put radio in standby before transmitting
    m_radio->standby();
    
    // Transmit the packet (blocking)
    Serial.println("[TX] Calling radio->transmit()...");
    unsigned long txStart = millis();
    int state = m_radio->transmit(const_cast<uint8_t*>(data), len);
    unsigned long txDuration = millis() - txStart;
    
    if (state != RADIOLIB_ERR_NONE) {
        m_lastError = "Transmit failed with code: " + String(state);
        Serial.printf("[TX] FAILED: %s (took %lu ms)\n", m_lastError.c_str(), txDuration);
        // Return to receive mode even on error
        startReceive();
        return false;
    }
    
    Serial.printf("[TX] Radio reported success (took %lu ms)\n", txDuration);
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
    
    return true;
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
        
        // Restart receive mode after error
        startReceive();
    }
    // For RADIOLIB_ERR_RX_TIMEOUT, no action needed - still in receive mode
}

#endif // HAS_LORA_RADIO
