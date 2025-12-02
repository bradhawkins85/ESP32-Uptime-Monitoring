# ESP32 Uptime Monitor

A basic uptime monitor written for the ESP32. It was written for an XDA article.

It serves as a framework to monitor services where support can be hardcoded as a type, making it expandable. Users can also add their own GET requests and ping requests, so that it can support any service.

## Features

- **HTTP GET** requests with expected response validation
- **Ping** monitoring
- **SNMP GET** checks with comparison operators (<, >, <=, >=, =, <>)
- **Pass/Fail Thresholds** - Configure how many consecutive successes or failures are required before changing a service's status and sending notifications
- Optional **ntfy offline notifications** when services go down
- Optional **Discord webhook notifications** for service up/down events
- Optional **SMTP email notifications** for service up/down events
- **RGB LED status indicator** - Visual feedback on system and service health
- **LCD and Touch Screen support** - Optional hardware display for viewing service status (on supported boards)
- Web-based UI for adding and managing services
- Persistent storage using LittleFS
- **Export/Import** monitor configurations for backup and restore
- **OTA Updates** - Update firmware via web interface without USB connection

## Supported Boards

The firmware supports multiple ESP32-S3 based boards. Select the appropriate environment when building with PlatformIO:

| Board | Environment | Display | MeshCore Transport | Description |
|-------|-------------|---------|-------------------|-------------|
| ESP32-S3 DevKitC-1 N16R8 | `esp32-n16r8` | None | BLE | Standard ESP32-S3 development board with 16MB flash. RGB LED for status indication. Connects to external MeshCore device via BLE. |
| Guition ESP32-4848S040 | `esp32-4848S040` | 480x480 LCD + Touch | BLE | ESP32-S3 with 4" square IPS display and GT911 touch controller. Service status is shown on the display. |
| Heltec Wireless Stick Lite V3 | `heltec-wireless-stick-lite-v3` | None | Built-in LoRa | ESP32-S3 with built-in SX1262 LoRa radio. Sends MeshCore messages directly over the radio without needing an external device. |


### Building for a Specific Board

To build for a specific board, use the `-e` flag with the environment name:

```bash
# For ESP32-S3 DevKitC-1 N16R8 (no display)
pio run -e esp32-n16r8

# For Guition ESP32-4848S040 (with LCD and touch)
pio run -e esp32-4848S040

# For Heltec Wireless Stick Lite V3 (with built-in LoRa radio)
pio run -e heltec-wireless-stick-lite-v3
```

### LCD Display Features (ESP32-4848S040)

When using the Guition ESP32-4848S040 board, the firmware provides:

- **Service Status Display**: Shows the current service name, status (UP/DOWN), host information, and last check time
- **Touch Navigation**: Tap the left half of the screen to view the previous service, tap the right half for the next service
- **Auto-Rotation**: Automatically cycles through services every 8 seconds
- **Visual Indicators**: Color-coded status (green for UP, red for DOWN), with error messages displayed when available

## RGB LED Status Indicator

The ESP32-S3 DevKitC has a built-in RGB LED that provides visual feedback on system status. The LED indicates the following states:

| LED Color | Pattern | Status |
|-----------|---------|--------|
| Blue | Pulsing | **Booting** - System is starting up and has not yet performed any service checks |
| Orange | Steady | **No WiFi** - Unable to connect to the configured WiFi network |
| White | Steady | **MeshCore** - Communicating with MeshCore radio over BLE |
| Green | Pulsing | **All Up** - All monitored services are online and healthy |
| Red | Pulsing | **Service Down** - One or more monitored services are offline |

### Disabling the LED

The LED is enabled by default. To disable it, set the following in `src/config.cpp`:

```cpp
#define LED_ENABLED_VALUE false
```

Or provide it as a PlatformIO build flag:

```ini
build_flags =
    -DLED_ENABLED_VALUE=false
```

## Prerequisites

Before deploying the code to your ESP32 board, you need to have PlatformIO installed. There are two main ways to use PlatformIO:

### Option 1: VS Code with PlatformIO Extension (Recommended)

1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Open VS Code and go to the Extensions view (Ctrl+Shift+X / Cmd+Shift+X)
3. Search for "PlatformIO IDE" and install it
4. Restart VS Code when prompted

### Option 2: PlatformIO Core (CLI)

Install PlatformIO Core using pip:

```bash
pip install platformio
```

## Configuration

The recommended way to configure the firmware is using a `.env` file, which keeps your sensitive values out of source control.

### Quick Start with .env File (Recommended)

1. Copy the example environment file:
   ```bash
   cp .env.example .env
   ```

2. Edit `.env` with your configuration values:
   ```bash
   # Required: WiFi credentials
   WIFI_SSID=YourNetworkName
   WIFI_PASSWORD=YourPassword

   # Optional: Web interface authentication
   WEB_AUTH_USERNAME=admin
   WEB_AUTH_PASSWORD=strong-password

   # Optional: Notification settings
   NTFY_TOPIC=my-alerts
   DISCORD_WEBHOOK_URL=https://discord.com/api/webhooks/...
   ```

3. Build and upload normally - values are automatically loaded from `.env`

The `.env` file is gitignored, so your secrets won't be committed to version control.

### Alternative: PlatformIO Build Flags

You can also provide configuration values directly in `platformio.ini`:

```ini
build_flags =
    -DWIFI_SSID_VALUE=\"YourNetworkName\"
    -DWIFI_PASSWORD_VALUE=\"YourPassword\"
```

### Alternative: Edit config.cpp Directly

For advanced users, you can edit `src/config.cpp` directly. Note that this file is gitignored to prevent accidentally committing secrets.

### Protecting the web interface

Optional HTTP Basic Authentication can be enabled to prevent unauthenticated users from adding, importing, or deleting services.

Add the following values to your `.env` file to require credentials when accessing the interface or modifying services:

```bash
WEB_AUTH_USERNAME=admin
WEB_AUTH_PASSWORD=strong-password
```

### Pairing with MeshCore over BLE

The firmware now acts as a BLE **client** that connects to the Heltec T114 (MeshCore) using its advertised name and pairing PIN, then writes alert messages to either a MeshCore channel, a Room Server, or both.

**Note:** This section applies to boards without built-in LoRa radio (e.g., `esp32-n16r8`, `esp32-4848S040`). For boards with built-in LoRa (e.g., `heltec-wireless-stick-lite-v3`), see the "Using Built-in LoRa Radio" section below.

1. **Set the MeshCore peer name, PIN, and destinations**
   - Add the following values to your `.env` file so the ESP32 knows how to reach the T114:
     ```bash
     BLE_PEER_NAME=Heltec-T114        # the T114's advertised name
     BLE_PAIRING_PIN=123456           # the MeshCore pairing PIN
     BLE_MESH_CHANNEL_NAME=alerts     # channel to use on the T114 (must already exist on the device)
     BLE_MESH_ROOM_SERVER_ID=         # Room Server public key (64 hex characters)
     BLE_MESH_ROOM_SERVER_PASSWORD=   # Room Server password for authentication
     BLE_DEVICE_NAME=ESP32-Uptime     # Optional: local name shown by the ESP32 itself
     ```
   - Alternatively, use PlatformIO build flags:
     ```ini
     ; platformio.ini
     -DBLE_PEER_NAME_VALUE=\"Heltec-T114\"
     -DBLE_PAIRING_PIN_VALUE=123456
     -DBLE_MESH_CHANNEL_NAME_VALUE=\"alerts\"
     -DBLE_MESH_ROOM_SERVER_ID_VALUE=\"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\"
     -DBLE_MESH_ROOM_SERVER_PASSWORD_VALUE=\"your-room-password\"
     -DBLE_DEVICE_NAME_VALUE=\"MyMeshBridge\"
     ```
   
   **Note:** The channel must be pre-configured on the MeshCore device before use. The ESP32 will not create or modify the channel. You can configure either a channel, a Room Server, or both. Messages will be sent to all configured destinations.

   **Room Server ID:** The Room Server ID is the 32-byte (64 hex character) public key of the Room Server node. You can obtain this from the MeshCore companion app or the Room Server's configuration. Messages sent to a Room Server are delivered as direct messages to that specific node.

   **Room Server Password:** Room Servers require authentication to post messages. Set `BLE_MESH_ROOM_SERVER_PASSWORD` to the password configured on the Room Server. The password is prepended to the message for authentication.

2. **Flash the firmware and watch the serial monitor**
   - On boot the ESP32 scans for the MeshCore name. Logs will show `MeshCore peer found, attempting connection...` and `Connected to MeshCore peer...` once the link is up. If the peer is missing or the PIN is wrong, the error message appears in `/api/mesh/status`.

3. **Allow the ESP32 to initiate the connection**
   - Put the T114/MeshCore radio in pairing mode so it advertises with the configured name and accepts the PIN above. The ESP32 will pair, bond, and stay connected; the T114 does **not** subscribe to the ESP32.

4. **Triggering alerts to MeshCore**
   - The ESP32 acts as a client: it scans for the T114, pairs, and then writes alerts to the configured destinations (channel and/or Room Server) over the MeshCore characteristic. The channel must already exist on the MeshCore device.
   - You can also send ad-hoc messages via the web API (the ESP32 will connect first if needed):
     ```bash
     curl -X POST http://<device-ip>/api/mesh/send \
       -u "<user>:<pass>" \
       -H "Content-Type: application/json" \
       -d '{"title":"Test","message":"Hello Mesh"}'
     ```
     Replace `<user>`/`<pass>` with your Basic Auth credentials (if enabled) and `<device-ip>` with the ESP32 IP address. The endpoint returns an error if the T114 cannot be found or paired.

### Using Built-in LoRa Radio (Heltec Wireless Stick Lite V3)

For the Heltec Wireless Stick Lite V3, the firmware uses the built-in SX1262 LoRa radio to send MeshCore messages directly without needing an external MeshCore device or BLE connection.

**Benefits of built-in LoRa:**
- No external MeshCore device required
- WiFi and LoRa can operate simultaneously (no coexistence issues like BLE+WiFi)
- Simpler setup - just configure radio parameters to match your mesh network
- Lower latency for notifications

1. **Configure LoRa radio parameters**
   - Add the following values to your `.env` file to match your MeshCore network settings:
     ```bash
     # LoRa radio configuration (must match your MeshCore network)
     LORA_FREQUENCY=915.0           # MHz (915.0 for US, 868.0 for EU)
     LORA_BANDWIDTH=250.0           # kHz (MeshCore default: 250.0)
     LORA_SPREADING_FACTOR=10       # 7-12 (MeshCore default: 10)
     LORA_CODING_RATE=5             # 5-8 for 4/5 to 4/8 (default: 5)
     LORA_SYNC_WORD=18              # MeshCore sync word (0x12 = 18 decimal)
     LORA_TX_POWER=22               # dBm, max 22 for SX1262
     
     # Channel name for message addressing (still used in packet format)
     BLE_MESH_CHANNEL_NAME=Alerts
     ```

2. **Build for the Heltec board**
   ```bash
   pio run -e heltec-wireless-stick-lite-v3 --target upload
   ```

3. **Messages are sent directly over LoRa**
   - When a service goes down/up, the notification is broadcast directly on the configured LoRa frequency
   - Other MeshCore nodes in range will receive the message
   - The web API `/api/mesh/send` works the same way but sends over LoRa instead of BLE

### Enabling ntfy notifications

Set an ntfy topic to receive alerts whenever a monitored service goes offline. Optional bearer or basic authentication is also supported for secured ntfy servers.

1. Add the following values to your `.env` file:
   ```bash
   NTFY_SERVER=https://ntfy.sh
   NTFY_TOPIC=esp32-uptime
   # use either a bearer token or basic auth credentials
   NTFY_ACCESS_TOKEN=my-token
   # NTFY_USERNAME=user
   # NTFY_PASSWORD=pass
   ```
2. Flash the firmware. The device will publish a message to the topic whenever it detects that a service transitioned to a down state, and another when the service comes back online.

### Enabling Discord notifications

Send alerts to a Discord channel by configuring a webhook URL.

1. Create a webhook in your target Discord channel (Channel Settings → Integrations → Webhooks) and copy the webhook URL.
2. Add the webhook URL to your `.env` file:
   ```bash
   DISCORD_WEBHOOK_URL=https://discord.com/api/webhooks/...
   ```
3. Rebuild and flash the firmware. A message will be sent to the webhook whenever a service changes state between up and down.

### Enabling SMTP notifications

Configure an SMTP relay to receive email alerts for uptime changes.

1. Add the following values to your `.env` file:
   ```bash
   SMTP_SERVER=smtp.example.com
   SMTP_PORT=587
   SMTP_USE_TLS=true
   SMTP_USERNAME=user@example.com
   SMTP_PASSWORD=password
   SMTP_FROM_ADDRESS=monitor@example.com
   SMTP_TO_ADDRESS=alerts@example.com
   ```
2. Rebuild and flash the firmware. The device will send an email whenever a service changes between up and down states. Multiple recipients can be provided as a comma-separated list.

### Enabling boot notifications

Optionally send a notification to all configured channels when the device boots up.

1. Add the following value to your `.env` file:
   ```bash
   BOOT_NOTIFICATION_ENABLED=true
   ```
2. Rebuild and flash the firmware. When the device boots and connects to WiFi, it will send a notification to all configured channels (ntfy, Discord, SMTP, MeshCore) indicating that the monitor has started and showing the device IP address.

## Deploying to ESP32

### Connect Your ESP32 Board

1. Connect your ESP32 board to your computer using a USB cable. Supported boards include:
   - ESP32-S3-DevKitC-1-N16R8 (environment: `esp32-n16r8`)
   - Guition ESP32-4848S040 with 480x480 LCD (environment: `esp32-4848S040`)
2. The board should be automatically detected by your system
3. On Linux, you may need to add your user to the `dialout` group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   Log out and back in for the changes to take effect.

### Using VS Code with PlatformIO IDE

1. Open the project folder in VS Code
2. Wait for PlatformIO to initialize (you'll see a progress indicator in the status bar)
3. Click the PlatformIO icon in the left sidebar
4. Under "Project Tasks", select your board environment (`esp32-n16r8` or `esp32-4848S040`):
   - Click **Build** to compile the firmware
   - Click **Upload** to flash the firmware to your ESP32
   - Click **Monitor** to view serial output

Alternatively, use the keyboard shortcuts:
- **Ctrl+Alt+B** (Cmd+Alt+B on Mac): Build
- **Ctrl+Alt+U** (Cmd+Alt+U on Mac): Upload

### Using PlatformIO CLI

Navigate to the project directory and run:

```bash
# Build the firmware (builds all environments by default)
pio run

# Build for a specific board
pio run -e esp32-n16r8         # For ESP32-S3 DevKitC (no display)
pio run -e esp32-4848S040      # For Guition ESP32-4848S040 (with LCD)

# Upload to the ESP32 (specify environment with -e)
pio run -e esp32-n16r8 --target upload

# Monitor serial output
pio device monitor
```

Or combine build and upload in one command:

```bash
pio run -e esp32-n16r8 --target upload && pio device monitor
```

## Monitoring Serial Output

After uploading, open the serial monitor to view debug information and the IP address of your ESP32:

- **VS Code**: Click "Monitor" in the PlatformIO sidebar
- **CLI**: Run `pio device monitor`

The monitor is configured to run at 115200 baud rate.

You should see output similar to:

```
Starting ESP32 Uptime Monitor...
LittleFS mounted successfully
Connecting to WiFi...
...
WiFi connected!
IP address: 192.168.1.100
Web server started
System ready!
Access web interface at: http://192.168.1.100
```

## Accessing the Web Interface

Once the ESP32 is running and connected to your WiFi:

1. Note the IP address shown in the serial monitor
2. Open a web browser on a device connected to the same network
3. Navigate to `http://<ESP32_IP_ADDRESS>` (e.g., `http://192.168.1.100`)
4. Use the web interface to add and manage monitoring services

## Backup and Restore Monitor Configurations

The web interface provides export and import functionality to backup and restore your monitor configurations. This is useful when:

- Performing firmware updates that may reset the filesystem
- Migrating configurations to a different ESP32 device
- Creating backups before making changes

### Exporting Monitors

1. Click the **Export Monitors** button in the web interface
2. A JSON file (`monitors-backup.json`) will be downloaded containing all your service configurations
3. Store this file safely for future restoration

### Importing Monitors

1. Click the **Import Monitors** button in the web interface
2. Select a previously exported JSON backup file
3. The services will be added to your current configuration
4. A message will indicate how many services were imported

**Note:** Importing adds services to existing ones rather than replacing them. If you want to start fresh, delete existing services before importing.

## OTA (Over-The-Air) Updates

The ESP32 Uptime Monitor supports firmware updates via the web interface, eliminating the need for a USB connection after initial deployment.

### Updating Firmware via OTA

1. Build your new firmware binary using PlatformIO:
   ```bash
   pio run
   ```
2. The compiled binary will be located at `.pio/build/esp32-n16r8/firmware.bin`
3. Open the web interface in your browser
4. Click the **OTA Update** button in the header (opens in a new tab)
5. On the ElegantOTA page, click "Firmware" and select your `firmware.bin` file
6. Click "Update" to begin the firmware upload
7. Wait for the upload to complete and the device to reboot

**Note:** Your monitor configurations (services) are stored in LittleFS and will persist across firmware updates.

### Generating OTA-compatible Binaries

For OTA updates, you only need the `firmware.bin` file. This is automatically generated when you build the project:

```bash
# Build firmware for your board
pio run -e esp32-n16r8         # For ESP32-S3 DevKitC
pio run -e esp32-4848S040      # For Guition ESP32-4848S040

# The binary is at:
# .pio/build/esp32-n16r8/firmware.bin       (for esp32-n16r8)
# .pio/build/esp32-4848S040/firmware.bin    (for esp32-4848S040)
```

### Security Considerations

- If HTTP Basic Authentication is enabled (via `WEB_AUTH_USERNAME` and `WEB_AUTH_PASSWORD`), the same credentials are required to access the OTA update page at `/update`
- If authentication is not configured, the OTA update page will be accessible without credentials
- Consider restricting network access to the ESP32 if security is a concern
- Always backup your monitor configurations before performing firmware updates

## Troubleshooting

### Upload Failed

- Ensure the USB cable supports data transfer (not just charging)
- Try a different USB port
- Hold the BOOT button on the ESP32 while initiating upload
- Check that no other application is using the serial port

### WiFi Connection Failed

- Verify your SSID and password are correct
- Ensure your WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
- Check that the ESP32 is within range of your router

### Serial Monitor Shows Garbage Characters

- Ensure the baud rate is set to 115200
- Try resetting the ESP32 board

## License

See the [LICENSE](LICENSE) file for details.
