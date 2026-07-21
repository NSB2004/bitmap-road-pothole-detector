/*
  ============================================================
  BitMap – Road Condition Sensor Node
  Device: ESP32 + MPU6050 + GPS + Ultrasonic Sensor
  Author: BitMap Team
  ============================================================

  HOW IT WORKS:
  1. MPU6050 reads accelerometer Z-axis 50 times/second
  2. If Z-axis spike > POTHOLE_THRESHOLD → pothole detected
  3. Severity score 1-10 is calculated from spike magnitude
  4. GPS gives lat/lon (simulated offset for demo mode)
  5. Data POSTed to Flask server over Wi-Fi

  WIRING SUMMARY:
  MPU6050:  VCC→3.3V | GND→GND | SCL→GPIO22 | SDA→GPIO21
  GPS:      VCC→3.3V | GND→GND | TX→GPIO16  | RX→GPIO17
  Ultrasonic: VCC→5V | GND→GND | TRIG→GPIO5 | ECHO→GPIO18
  ============================================================
*/

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ─────────────── CONFIGURE THESE BEFORE UPLOADING ───────────────
const char* WIFI_SSID     = "YOUR_PHONE_HOTSPOT_NAME";   // Your phone hotspot name
const char* WIFI_PASSWORD = "YOUR_HOTSPOT_PASSWORD";     // Your phone hotspot password
const char* SERVER_URL    = "http://192.168.X.X:5000/api/report"; // Replace X.X with your laptop IP
// ─────────────────────────────────────────────────────────────────

// ── MPU6050 Registers ──
#define MPU6050_ADDR     0x68
#define REG_PWR_MGMT     0x6B
#define REG_ACCEL_XOUT_H 0x3B

// ── Pin Definitions ──
#define ULTRASONIC_TRIG  5
#define ULTRASONIC_ECHO  18
#define ONBOARD_LED      2    // ESP32 built-in LED (blinks on detect)

// ── Pothole Detection Settings ──
#define POTHOLE_THRESHOLD 1.8  // G-force spike to count as pothole (lower = more sensitive)
                               // For hand-shake demo: 1.5 works well
                               // For real vehicle: try 2.5
#define COOLDOWN_MS       2000 // 2 seconds between detections (avoid duplicate reports)

// ── Demo Mode GPS ──
// For the hand-shake demo, we use a fixed base location with small random offsets
// This simulates a vehicle moving around. Replace with real GPS when available.
#define DEMO_MODE         true   // Set to false when real GPS module is connected
#define BASE_LAT          12.9716 // Change to YOUR city's coordinates
#define BASE_LON          77.5946 // e.g. Bengaluru: 12.9716, 77.5946

// ── Global State ──
float baseline_z = 1.0;       // Calibrated baseline (set during setup)
unsigned long last_detect = 0; // Timestamp of last detection
int report_count = 0;         // Total potholes reported this session

// ─────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n============================");
  Serial.println("  BitMap Sensor Node v1.0");
  Serial.println("============================");

  // LED setup
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);

  // Ultrasonic pins
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // ── Init MPU6050 ──
  Wire.begin(21, 22);  // SDA=21, SCL=22
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_PWR_MGMT);
  Wire.write(0);  // Wake up
  Wire.endTransmission(true);
  Serial.println("[MPU6050] Initialized OK");

  // ── Calibrate Baseline ──
  Serial.println("[Calibrate] Hold sensor STILL for 3 seconds...");
  delay(1000);
  baseline_z = calibrateBaseline();
  Serial.print("[Calibrate] Baseline Z = ");
  Serial.println(baseline_z);

  // ── Connect Wi-Fi ──
  Serial.print("[WiFi] Connecting to ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] ESP32 IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] FAILED! Check SSID/password");
  }

  Serial.println("\n[Ready] Shake the sensor hard to simulate a pothole!");
  Serial.println("[Ready] Monitoring started...\n");
}

// ─────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────
void loop() {
  float ax, ay, az;
  readAccelerometer(ax, ay, az);

  // Calculate spike from baseline
  float z_delta = abs(az - baseline_z);

  // Print live readings to Serial Monitor (useful for testing)
  Serial.print("Z="); Serial.print(az, 2);
  Serial.print(" | Delta="); Serial.print(z_delta, 2);
  Serial.print(" | Threshold="); Serial.println(POTHOLE_THRESHOLD);

  // ── Pothole Detection ──
  unsigned long now = millis();
  if (z_delta > POTHOLE_THRESHOLD && (now - last_detect) > COOLDOWN_MS) {
    last_detect = now;
    report_count++;

    // Calculate severity (1–10 scale)
    int severity = calculateSeverity(z_delta);

    // Get location
    float lat, lon;
    getLocation(lat, lon);

    // Measure depth with ultrasonic (optional)
    float depth_cm = measureDepth();

    // Flash LED to confirm detection
    flashLED(severity);

    // Print to Serial Monitor
    Serial.println("\n *** POTHOLE DETECTED! ***");
    Serial.print("   Severity : "); Serial.print(severity); Serial.println("/10");
    Serial.print("   Delta-Z  : "); Serial.println(z_delta);
    Serial.print("   Depth    : "); Serial.print(depth_cm); Serial.println(" cm");
    Serial.print("   Location : "); Serial.print(lat, 6); Serial.print(", "); Serial.println(lon, 6);
    Serial.print("   Total    : #"); Serial.println(report_count);

    // Send to server
    sendReport(lat, lon, severity, z_delta, depth_cm);

    Serial.println(" Resuming monitoring...\n");
  }

  delay(20); // 50Hz sampling rate
}

// ─────────────────────────────────────────────────────────────────
// MPU6050 FUNCTIONS
// ─────────────────────────────────────────────────────────────────

// Read raw accelerometer values and convert to G-force
void readAccelerometer(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  int16_t raw_x = (Wire.read() << 8) | Wire.read();
  int16_t raw_y = (Wire.read() << 8) | Wire.read();
  int16_t raw_z = (Wire.read() << 8) | Wire.read();

  // Default sensitivity: ±2g → divide by 16384
  ax = raw_x / 16384.0;
  ay = raw_y / 16384.0;
  az = raw_z / 16384.0;
}

// Average 100 readings to set baseline
float calibrateBaseline() {
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    float ax, ay, az;
    readAccelerometer(ax, ay, az);
    sum += az;
    delay(10);
  }
  return sum / 100.0;
}

// ─────────────────────────────────────────────────────────────────
// SEVERITY CALCULATION
// ─────────────────────────────────────────────────────────────────

int calculateSeverity(float delta) {
  // Maps delta G-force to severity score 1-10
  // delta < 2.0  → score 1-3  (minor bump)
  // delta 2–4   → score 4-6  (moderate pothole)
  // delta > 4   → score 7-10 (severe damage)
  if (delta < POTHOLE_THRESHOLD) return 0;
  float score = (delta - POTHOLE_THRESHOLD) * 3.0 + 1.0;
  return constrain((int)score, 1, 10);
}

// ─────────────────────────────────────────────────────────────────
// GPS / LOCATION
// ─────────────────────────────────────────────────────────────────

void getLocation(float &lat, float &lon) {
  if (DEMO_MODE) {
    // Add small random offset to simulate vehicle movement
    // Range: ~50–100 metres in each direction
    float lat_offset = ((random(-100, 100)) / 10000.0);
    float lon_offset = ((random(-100, 100)) / 10000.0);
    lat = BASE_LAT + lat_offset;
    lon = BASE_LON + lon_offset;
  } else {
    // TODO: Replace with real GPS parsing (TinyGPS++ library)
    // Example with TinyGPS++:
    // while (gpsSerial.available()) gps.encode(gpsSerial.read());
    // lat = gps.location.lat();
    // lon = gps.location.lng();
    lat = BASE_LAT;
    lon = BASE_LON;
  }
}

// ─────────────────────────────────────────────────────────────────
// ULTRASONIC SENSOR (Depth Measurement)
// ─────────────────────────────────────────────────────────────────

float measureDepth() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // 30ms timeout
  float distance_cm = (duration * 0.0343) / 2.0;

  // If sensor reads 0 or timeout, return -1 (not available)
  if (duration == 0 || distance_cm > 400) return -1.0;
  return distance_cm;
}

// ─────────────────────────────────────────────────────────────────
// HTTP REPORT TO FLASK SERVER
// ─────────────────────────────────────────────────────────────────

void sendReport(float lat, float lon, int severity, float delta_z, float depth_cm) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] Not connected to WiFi! Skipping upload.");
    return;
  }

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");

  // Build JSON payload
  StaticJsonDocument<256> doc;
  doc["lat"]       = lat;
  doc["lon"]       = lon;
  doc["severity"]  = severity;
  doc["delta_z"]   = delta_z;
  doc["depth_cm"]  = depth_cm;
  doc["device_id"] = "ESP32_DEMO_01";

  String payload;
  serializeJson(doc, payload);

  Serial.print("[HTTP] Sending to server: ");
  Serial.println(payload);

  int code = http.POST(payload);
  if (code == 200) {
    Serial.println("[HTTP] ✓ Server received report!");
  } else {
    Serial.print("[HTTP] ✗ Error code: ");
    Serial.println(code);
    Serial.println("[HTTP] Check: Is Flask server running? Is IP correct?");
  }
  http.end();
}

// ─────────────────────────────────────────────────────────────────
// VISUAL FEEDBACK
// ─────────────────────────────────────────────────────────────────

void flashLED(int severity) {
  // Flash LED: more flashes = higher severity
  int flashes = min(severity, 5);
  for (int i = 0; i < flashes; i++) {
    digitalWrite(ONBOARD_LED, HIGH);
    delay(100);
    digitalWrite(ONBOARD_LED, LOW);
    delay(100);
  }
}