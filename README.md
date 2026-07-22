# BitMap – Road Condition Sensor Node 🚗📡

BitMap (also developed under the team name ROADAR) is an IoT-based road monitoring system built to detect, measure, and map road anomalies like potholes and bumps in real-time. Designed around the ESP32, this system turns any vehicle into a mobile infrastructure scanner.

## 🚀 How It Works
1. **Motion Tracking:** An MPU6050 sensor reads the accelerometer Z-axis at 50Hz.
2. **Anomaly Detection:** If a Z-axis spike exceeds the predefined threshold (e.g., hitting a pothole), it registers as an anomaly.
3. **Severity & Depth:** The system calculates a severity score (1-10) based on the G-force spike and uses an HC-SR04 Ultrasonic sensor to measure the physical depth of the pothole.
4. **Geolocation:** A GPS module logs the exact latitude and longitude of the anomaly.
5. **Data Transmission:** The ESP32 POSTs a JSON payload containing location, severity, and depth data to a local Flask server via Wi-Fi for live mapping.

## 🛠️ Tech Stack & Hardware
*   **Microcontroller:** ESP32
*   **Sensors:** MPU6050 (Accelerometer/Gyro), HC-SR04 (Ultrasonic), GPS Module
*   **Software/Libraries:** Arduino IDE (C++), `Wire.h`, `WiFi.h`, `HTTPClient.h`, `ArduinoJson`
*   **Backend:** Flask (Python)

## 🔌 Wiring Summary

| Component | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **MPU6050** | `GPIO21` (SDA), `GPIO22` (SCL) | Powered via 3.3V |
| **GPS Module**| `GPIO16` (TX), `GPIO17` (RX) | Powered via 3.3V |
| **HC-SR04** | `GPIO5` (TRIG), `GPIO18` (ECHO) | Powered via 5V |
| **Onboard LED**| `GPIO2` | Flashes based on severity |

## ⚙️ Setup & Configuration
1. Open the `.ino` file in the Arduino IDE.
2. Install the `ArduinoJson` library via the Library Manager.
3. Update the following network credentials at the top of the code:
   ```cpp
   const char* WIFI_SSID     = "YOUR_PHONE_HOTSPOT_NAME";
   const char* WIFI_PASSWORD = "YOUR_HOTSPOT_PASSWORD";
   const char* SERVER_URL    = "[http://192.168.](http://192.168.)X.X:5000/api/report"; // Your Flask server IP