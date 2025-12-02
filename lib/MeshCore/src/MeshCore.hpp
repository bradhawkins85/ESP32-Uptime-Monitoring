#pragma once

/**
 * MeshCore - Layered implementation of MeshCore Companion Radio Protocol
 * 
 * Architecture:
 * 
 *   [Application Layer]
 *           |
 *   [CompanionProtocol] - MeshCore protocol logic, state machine, API
 *           |
 *   [FrameCodec]        - Frame parsing and building
 *           |
 *   [Transport : IByteTransport] - BLE or LoRa connection and I/O
 * 
 * Transport options:
 *   - BLECentralTransport: Connects to an external MeshCore device via BLE
 *   - LoRaTransport: Uses built-in SX1262 radio (e.g., Heltec Wireless Stick Lite V3)
 * 
 * Usage (BLE mode - default):
 *   1. Create BLECentralTransport with configuration
 *   2. Create FrameCodec with the transport
 *   3. Create CompanionProtocol with both
 *   4. Initialize BLE, connect, start session
 *   5. Find channel and send messages
 * 
 * Usage (LoRa mode - HAS_LORA_RADIO defined):
 *   1. Create LoRaTransport with pin configuration
 *   2. Create FrameCodec with the transport
 *   3. Initialize radio, send messages directly
 *   Note: No session/channel lookup needed - messages go directly to mesh
 */

#include "IByteTransport.hpp"
#include "FrameCodec.hpp"

// Include transport implementations based on build configuration
#ifdef HAS_LORA_RADIO
#include "LoRaTransport.hpp"
#else
#include "BLECentralTransport.hpp"
#include "CompanionProtocol.hpp"
#endif
