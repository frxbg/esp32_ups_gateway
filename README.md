<a name="readme-top"></a>

<!-- PROJECT SHIELDS -->
<!--
*** Using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
-->
[![Version][version-shield]][version-url]
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]

<br />
<div align="center">
  <a href="https://github.com/yourusername/esp32-ups-gateway">
    <img src="https://raw.githubusercontent.com/Tarikul-Islam-Anik/Animated-Fluent-Emojis/master/Emojis/Objects/Battery.png" alt="Logo" width="80" height="80">
  </a>

  <h3 align="center">ESP32 UPS Gateway</h3>

  <p align="center">
    A professional industrial gateway to monitor CyberPower UPS devices via USB HID, ModbusRTU, and a modern Web Dashboard.
    <br />
    <a href="#usage"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://raw.githack.com/frxbg/esp32_ups_gateway/refs/heads/main/assets/demo.html">View Demo</a>
    ·
    <a href="#issues">Report Bug</a>
    ·
    <a href="#issues">Request Feature</a>
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About The Project

![Product Screenshot](https://github.com/frxbg/esp32_ups_gateway/blob/main/assets/Dashboard_demo.png)

There are many UPS monitoring solutions out there, but most require a dedicated PC or Raspberry Pi. The **ESP32 UPS Gateway** is a microcontroller-based solution that bridges the gap between your USB-connected UPS and your industrial or home automation network.

It acts as a **ModbusRTU Slave**, allowing PLCs and SCADA systems to read battery status, voltages, and load data directly. Simultaneously, it hosts a **responsive Web Dashboard** for easy human monitoring and configuration.

**Key Features:**
*   **USB HID Host**: Driverless connection to CyberPower UPS (and compatible models).
*   **Dual Interface**: ModbusRTU (RS-485/TTL) + HTTP Web Server.
*   **Smart Connectivity**: WiFi Station mode with automatic AP fallback (`UPS_Gateway_AP`).
*   **Configuration**: Change Modbus settings (Baud, Parity, Stop Bits) via the Web UI.
*   **Robustness**: Watchdog timers, persistent storage (NVS), and auto-reconnection logic.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

This project is built using the robust Espressif IoT Development Framework.

*   [![ESP-IDF][ESP-IDF]][ESP-IDF-url]
*   [![C][C]][C-url]
*   [![HTML5][HTML5]][HTML5-url]
*   [![CSS3][CSS3]][CSS3-url]
*   [![JavaScript][JavaScript]][JavaScript-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running, follow these simple steps.

### Prerequisites

You will need the following hardware and software:

*   **Hardware**:
    *   ESP32-S3 Development Board (must support USB OTG).
    *   CyberPower UPS (tested with CP1600EPFCLCD).
    *   USB-A to USB-C/Micro cable.
    *   (Optional) RS-485 to TTL Module for Modbus.
    *   (Optional) WS2812B RGB LED for status indication.
*   **Software**:
    *   ESP-IDF v5.x installed.
    *   Git.

### Installation

1.  **Clone the repo**
    ```sh
    git clone https://github.com/yourusername/esp32-ups-gateway.git
    cd esp32-ups-gateway
    ```
2.  **Set up environment** (if not already done)
    ```sh
    . $HOME/esp/esp-idf/export.sh
    ```
3.  **Configure the project** (Optional)
    ```sh
    idf.py menuconfig
    ```
    *   Navigate to `Modbus Example Configuration` to set default UART pins.
4.  **Build and Flash**
    ```sh
    idf.py build
    idf.py -p COM3 flash monitor
    ```
    *(Replace `COM3` with your actual serial port)*

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

### 🖥️ Web Dashboard

The device hosts a Single Page Application (SPA) accessible via WiFi.

1.  Connect to the Access Point: **SSID**: `UPS_Gateway_AP`, **Pass**: `frigotehnica`.
2.  Navigate to `http://192.168.4.1`.
3.  **Dashboard Tab**: View real-time gauges for Battery, Load, and Voltages.
4.  **WiFi Tab**: Scan for local networks and connect the device to your home WiFi.
5.  **Modbus Tab**: Configure RS-485 settings (Address, Baud, Parity, Stop Bits) and view the Register Map.

### 🔌 ModbusRTU Interface

Integrate with Home Assistant or Industrial PLCs.

*   **Default Settings**: Address `1`, Baud `115200`, 8N1.
*   **Register Map**:
    | Address | Parameter | Unit | Type |
    |---|---|---|---|
    | 0 | Battery Charge | % | R |
    | 1 | Battery Runtime | s | R |
    | 2 | Input Voltage | V | R |
    | 3 | Output Voltage | V | R |
    | 4 | UPS Load | % | R |
    | 32 | Beeper Control | Enum | R/W |

*See [MODBUS_REGISTERS_FULL.md](MODBUS_REGISTERS_FULL.md) for complete documentation.*

### 🚦 LED Status

*   🟣 **Purple**: Booting
*   🟢 **Green**: Online (AC Present)
*   🔴 **Red**: Battery Mode (Power Outage)
*   🟠 **Orange**: Disconnected

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ROADMAP -->
## Roadmap

- [x] USB HID Communication
- [x] Web Dashboard (SPA)
- [x] ModbusRTU Slave
- [x] WiFi AP + Station Mode
- [x] Web-based Configuration
- [ ] MQTT Support
- [ ] Home Assistant Auto-Discovery
- [ ] Email/Telegram Notifications
- [ ] Historical Data Logging (SD Card)

See the [open issues](https://github.com/yourusername/esp32-ups-gateway/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1.  Fork the Project
2.  Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3.  Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4.  Push to the Branch (`git push origin feature/AmazingFeature`)
5.  Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Your Name - [@yourtwitter](https://twitter.com/yourtwitter) - email@example.com

Project Link: [https://github.com/yourusername/esp32-ups-gateway](https://github.com/yourusername/esp32-ups-gateway)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

*   [Best-README-Template](https://github.com/othneildrew/Best-README-Template)
*   [Espressif IoT Development Framework](https://github.com/espressif/esp-idf)
*   [CyberPower Systems](https://www.cyberpowersystems.com/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[version-shield]: https://img.shields.io/badge/version-1.0.0-blue.svg?style=for-the-badge
[version-url]: #
[contributors-shield]: https://img.shields.io/github/contributors/yourusername/esp32-ups-gateway.svg?style=for-the-badge
[contributors-url]: https://github.com/yourusername/esp32-ups-gateway/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/yourusername/esp32-ups-gateway.svg?style=for-the-badge
[forks-url]: https://github.com/yourusername/esp32-ups-gateway/network/members
[stars-shield]: https://img.shields.io/github/stars/yourusername/esp32-ups-gateway.svg?style=for-the-badge
[stars-url]: https://github.com/yourusername/esp32-ups-gateway/stargazers
[issues-shield]: https://img.shields.io/github/issues/yourusername/esp32-ups-gateway.svg?style=for-the-badge
[issues-url]: https://github.com/yourusername/esp32-ups-gateway/issues
[license-shield]: https://img.shields.io/github/license/yourusername/esp32-ups-gateway.svg?style=for-the-badge
[license-url]: https://github.com/yourusername/esp32-ups-gateway/blob/master/LICENSE
[ESP-IDF]: https://img.shields.io/badge/ESP--IDF-E7352C?style=for-the-badge&logo=espressif&logoColor=white
[ESP-IDF-url]: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/
[C]: https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white
[C-url]: https://en.wikipedia.org/wiki/C_(programming_language)
[HTML5]: https://img.shields.io/badge/HTML5-E34F26?style=for-the-badge&logo=html5&logoColor=white
[HTML5-url]: https://developer.mozilla.org/en-US/docs/Web/Guide/HTML/HTML5
[CSS3]: https://img.shields.io/badge/CSS3-1572B6?style=for-the-badge&logo=css3&logoColor=white
[CSS3-url]: https://developer.mozilla.org/en-US/docs/Web/CSS
[JavaScript]: https://img.shields.io/badge/JavaScript-F7DF1E?style=for-the-badge&logo=javascript&logoColor=black
[JavaScript-url]: https://developer.mozilla.org/en-US/docs/Web/JavaScript
