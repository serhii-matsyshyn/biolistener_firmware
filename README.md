# BioListener Firmware

**BioListener** is based on the **ESP32 D1 MINI** module (ESP-WROOM-32).

The firmware utilizes **PlatformIO** and the **Arduino framework** to enable easy and user-friendly code development. This setup provides seamless integration with open-source Arduino Framework developments and allows for easy adaptation to specific use cases.

<p align="center">
  <img alt="BioListener Boards v1.0" src="data/images/BioListener_boards_v1.0_prototypes.jpg" width="600">
</p>

<p align="center">
  <img alt="BioListener Boards v1.0" src="data/images/BioListener_boards_v1.0_esp32_prototypes.jpg" width="600">
</p>

## Communication Methods

### Currently Supported
- **WiFi TCP**: Real-time biosignal data transmission to the **BrainFlow server**.

### Planned
The following features are planned for future updates:
- **Bluetooth / BLE**: Real-time data transmission over Bluetooth / BLE support.
- **Offline Mode**: Data storage on **SD card**.
- **Unstable Networks Support**: Mode to handle unreliable network connections.
- **Big Data IoT Server**: Real-time data capture and processing from many boards.

## Useful Information For Developers
- Install the **PlatformIO** extension in **Visual Studio Code**.
- Use `platformio.ini` to configure the project settings and set build flags (specified below):
  - **ADC_USED**:  0: ADC_ADS131M08, 1: ADC_AD7771
  - **IMU_DISABLE**: 0: IMU_ENABLE, 1: IMU_DISABLE
  - **SERVER_IP**: Set BioListener server IP address for BrainFlow server.
  - **WIFI_SSID**: Set system variable WIFI_SSID to your WiFi SSID for secure configuration.
  - **WIFI_PASSWORD**: Set WIFI_PASSWORD to your WiFi password for secure configuration.
- Build the project using the **PlatformIO** extension in **Visual Studio Code**.
- Upload the firmware to the **ESP32 D1 MINI** module using the **PlatformIO** extension in **Visual Studio Code**.

## License

This repository uses the following licenses:
- **Code**: The code in this repository is licensed under the [GNU General Public License v3.0 (GPL-3.0)](https://www.gnu.org/licenses/gpl-3.0.html).
