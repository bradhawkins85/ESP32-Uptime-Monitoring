#include "FrameCodec.hpp"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define static constexpr members for pre-C++17 ODR compliance
constexpr size_t FrameCodec::MAX_FRAME_PAYLOAD;

FrameCodec::FrameCodec(IByteTransport& transport)
    : m_transport(transport)
{
    // Set up the transport to forward received bytes to this codec
    m_transport.setRxCallback([this](const uint8_t* data, size_t len) {
        onBytes(data, len);
    });
}

void FrameCodec::setFrameCallback(FrameCallback callback) {
    m_frameCallback = std::move(callback);
}

void FrameCodec::clearCallbacks() {
    m_frameCallback = nullptr;
    m_transport.clearCallbacks();
}

bool FrameCodec::sendFrame(uint8_t cmd, const uint8_t* payload, size_t payloadLen) {
    // Validate total frame size
    size_t totalLen = 1 + payloadLen;  // cmd + payload
    if (totalLen > MAX_FRAME_PAYLOAD) {
        Serial.printf("FrameCodec: frame too large (%d > %d)\n", (int)totalLen, (int)MAX_FRAME_PAYLOAD);
        return false;
    }

    // Build frame buffer
    std::vector<uint8_t> frame;
    frame.reserve(totalLen);
    frame.push_back(cmd);
    
    if (payload != nullptr && payloadLen > 0) {
        frame.insert(frame.end(), payload, payload + payloadLen);
    }

    Serial.printf("FrameCodec TX: cmd 0x%02X, payload length %d\n", cmd, (int)payloadLen);
    
    // Yield to scheduler before transport send to ensure system watchdog is fed.
    // This prevents TG0WDT_SYS_RST (Timer Group 0 Watchdog Timer System Reset)
    // when sending messages, especially after channel lookup operations that
    // may have accumulated processing time without yielding.
    vTaskDelay(pdMS_TO_TICKS(5));
    
    return m_transport.send(frame.data(), frame.size());
}

bool FrameCodec::sendFrame(uint8_t cmd, const std::vector<uint8_t>& payload) {
    return sendFrame(cmd, payload.empty() ? nullptr : payload.data(), payload.size());
}

void FrameCodec::onBytes(const uint8_t* data, size_t len) {
    // NOTE: This callback runs in BLE task context with limited stack (~3-4KB).
    // Avoid Serial.printf and other stack-heavy operations to prevent overflow.
    // If logging is needed, use simple Serial.print/println sparingly.
    
    // For BLE, each notification is a complete frame
    // First byte is the command/response code
    
    if (len == 0 || len > MAX_FRAME_PAYLOAD) {
        return;
    }

    uint8_t cmd = data[0];
    const uint8_t* payload = (len > 1) ? (data + 1) : nullptr;
    size_t payloadLen = (len > 1) ? (len - 1) : 0;

    if (m_frameCallback) {
        m_frameCallback(cmd, payload, payloadLen);
    }
}
