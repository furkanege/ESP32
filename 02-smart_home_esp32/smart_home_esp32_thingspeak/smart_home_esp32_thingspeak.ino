/*
  Project: Smart Home Monitor – ESP32 + ThingSpeak
  File: smart_home_esp32_thingspeak.ino
  Author: Furkan Ege
  Board: ESP32 Dev Module (NodeMCU-32S)
  Version: 1.0
  Last Updated: 01/08/2026

  Sensors:
    - AHT10: Temperature (°C) + Humidity (%)
    - MPU6050: Motion/shake detection
    - MQ135 (analog): Raw ADC + derived Air % (relative)

  OLED pages (auto-rotate):
    - TEMP: °C and °F + Humidity
    - AIR:  Air % + MQ raw
    - MOTION: X/Y/Z + shake YES/NO

  ThingSpeak fields:
    field1 = tempC
    field2 = humP
    field3 = airPct
    field4 = shake
    field5 = mqRaw
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <math.h>
#include <string.h>

const char* WIFI_SSID = "WIFI_NAME";
const char* WIFI_PASSWORD = "WIFI_PASSWORD";

const char* THINGSPEAK_URL = "http://api.thingspeak.com/update";
const char* THINGSPEAK_API_KEY = "YOUR_THINGSPEAK_WRITE_API";
const uint32_t THINGSPEAK_SEND_INTERVAL_MS = 5000;

const int I2C_SDA = 21;
const int I2C_SCL = 22;

const int MQ_PIN = 34;

const uint32_t OLED_PAGE_INTERVAL_MS = 3000;

#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool displayOk = false;

#define AHT10_ADDR 0x38
#define MPU6050_ADDR 0x68

byte page = 0;
const byte PAGE_COUNT = 3;
unsigned long lastPageMs = 0;

bool hasAHT = false;
bool hasMPU = false;

float tempC = 25.0f;
float humP = 50.0f;

int mqRaw = 0;
int mqBase = 0;
int mqDirty = 0;
const int MQ_SPAN = 250;
const int MQ_FLOOR_PCT = 1;

int16_t ax = 0, ay = 0, az = 0;
static bool accInitialized = false;
static float axE = 0, ayE = 0, azE = 0;
const float ACC_EMA_ALPHA = 0.10f;
static float magE = 0;
static float dynE = 0;
static float noiseE = 0;
const float MAG_ALPHA = 0.02f;
const float DYN_ALPHA = 0.25f;
const float NOISE_ALPHA = 0.01f;
const int MIN_QUAKE_COUNTS = 70;
const float NOISE_MULT = 5.0f;
unsigned long shakeUntilMs = 0;
const unsigned long SHAKE_HOLD_MS = 300;

static inline bool isShakeActive() { return millis() < shakeUntilMs; }

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline void printCentered(const char* text, int16_t y, uint8_t size) {
  display.setTextSize(size);
  int16_t x = (SCREEN_W - (int)strlen(text) * 6 * size) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, y);
  display.print(text);
}

bool i2cPresent(byte addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void readAHT10() {
  Wire.beginTransmission(AHT10_ADDR);
  Wire.write(0xAC);
  Wire.write(0x33);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(80);
  Wire.requestFrom(AHT10_ADDR, 6);
  if (Wire.available() != 6) return;
  byte data[6];
  for (int i = 0; i < 6; i++) data[i] = Wire.read();
  if (data[0] & 0x80) return;
  uint32_t h_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
  uint32_t t_raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
  humP = (h_raw * 100.0f) / 1048576.0f;
  tempC = ((t_raw * 200.0f) / 1048576.0f) - 50.0f;
  humP = clampf(humP, 0.0f, 100.0f);
  tempC = clampf(tempC, -40.0f, 85.0f);
}

void initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void readMPU6050Raw() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  if (Wire.available() < 6) return;
  ax = ((int16_t)Wire.read() << 8) | Wire.read();
  ay = ((int16_t)Wire.read() << 8) | Wire.read();
  az = ((int16_t)Wire.read() << 8) | Wire.read();
}

void updateMotion() {
  if (!hasMPU) return;
  readMPU6050Raw();
  if (!accInitialized) {
    axE = ax; ayE = ay; azE = az;
    magE = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az);
    dynE = 0; noiseE = 0;
    accInitialized = true;
    return;
  }
  axE = (1.0f - ACC_EMA_ALPHA) * axE + ACC_EMA_ALPHA * (float)ax;
  ayE = (1.0f - ACC_EMA_ALPHA) * ayE + ACC_EMA_ALPHA * (float)ay;
  azE = (1.0f - ACC_EMA_ALPHA) * azE + ACC_EMA_ALPHA * (float)az;
  float mag = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az);
  magE = (1.0f - MAG_ALPHA) * magE + MAG_ALPHA * mag;
  float dyn = fabsf(mag - magE);
  dynE = (1.0f - DYN_ALPHA) * dynE + DYN_ALPHA * dyn;
  if (!isShakeActive()) noiseE = (1.0f - NOISE_ALPHA) * noiseE + NOISE_ALPHA * dynE;
  float th = noiseE * NOISE_MULT;
  if (th < (float)MIN_QUAKE_COUNTS) th = (float)MIN_QUAKE_COUNTS;
  if (dynE > th) shakeUntilMs = millis() + SHAKE_HOLD_MS;
}

void calibrateMQBaseline() {
  long sum = 0;
  for (int i = 0; i < 220; i++) {
    sum += analogRead(MQ_PIN);
    delay(5);
  }
  mqBase = (int)(sum / 220);
  mqDirty = constrain(mqBase + MQ_SPAN, mqBase + 1, 1023);
}

int calcAirPct(int raw) {
  if (mqDirty <= mqBase) return MQ_FLOOR_PCT;
  raw = constrain(raw, mqBase, mqDirty);
  long pct = (long)(raw - mqBase) * 100L / (long)(mqDirty - mqBase);
  int ip = (int)constrain((int)pct, 0, 100);
  if (ip == 0) ip = MQ_FLOOR_PCT;
  return ip;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - t0 > 15000) break;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection timeout");
  }
}

void sendToThingSpeak() {
  if (WiFi.status() != WL_CONNECTED) return;
  const int airPct = calcAirPct(mqRaw);
  const int shake = isShakeActive() ? 1 : 0;
  String url = String(THINGSPEAK_URL) + "?api_key=" + THINGSPEAK_API_KEY;
  if (hasAHT) {
    url += "&field1=" + String(tempC, 1);
    url += "&field2=" + String(humP, 1);
  }
  url += "&field3=" + String(airPct);
  url += "&field4=" + String(shake);
  url += "&field5=" + String(mqRaw);
  HTTPClient http;
  WiFiClient client;
  http.begin(client, url);
  const int httpCode = http.GET();
  if (httpCode > 0) {
    Serial.printf("ThingSpeak HTTP:%d | T=%.1fC H=%.1f%% Air=%d%% Shake=%d MQ=%d\n",
                  httpCode, tempC, humP, airPct, shake, mqRaw);
  } else {
    Serial.printf("ThingSpeak error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

unsigned long lastSendMs = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);
  analogSetAttenuation(ADC_11db);
  Wire.begin(I2C_SDA, I2C_SCL);
  displayOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (displayOk) {
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.clearDisplay();
    printCentered("SMART", 0, 2);
    printCentered("HOME", 16, 2);
    printCentered("by FURKAN EGE", 34, 1);
    printCentered("ESP32 + TS", 44, 1);
    printCentered("Boston, MA", 54, 1);
    display.display();
    delay(1600);
  }
  hasAHT = i2cPresent(AHT10_ADDR);
  hasMPU = i2cPresent(MPU6050_ADDR);
  if (hasMPU) initMPU6050();
  calibrateMQBaseline();
  connectWiFi();
  lastSendMs = millis() - THINGSPEAK_SEND_INTERVAL_MS;
  lastPageMs = millis();
}

void loop() {
  const unsigned long now = millis();
  if (now - lastPageMs >= OLED_PAGE_INTERVAL_MS) {
    lastPageMs = now;
    page = (page + 1) % PAGE_COUNT;
  }
  if (hasAHT) readAHT10();
  mqRaw = analogRead(MQ_PIN);
  updateMotion();
  if (now - lastSendMs >= THINGSPEAK_SEND_INTERVAL_MS) {
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    sendToThingSpeak();
    lastSendMs = now;
  }
  if (!displayOk) { delay(20); return; }
  display.clearDisplay();
  if (page == 0) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println(F("TEMP"));
    if (hasAHT) {
      const int tC = (int)roundf(tempC);
      const int tF = (int)roundf((tempC * 9.0f / 5.0f) + 32.0f);
      char buf[20];
      const int n = snprintf(buf, sizeof(buf), "%dC %dF", tC, tF);
      if (n <= 7) printCentered(buf, 22, 3);
      else printCentered(buf, 26, 2);
      display.setTextSize(1);
      display.setCursor(0, 56);
      display.print(F("Humidity: "));
      display.print(humP, 1);
      display.print(F("%"));
    } else {
      display.setTextSize(2);
      printCentered("NO SENSOR", 28, 2);
    }
  } else if (page == 1) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println(F("AIR"));
    display.setTextSize(1);
    display.setCursor(0, 16);
    display.println(F("MQ135"));
    const int airPct = calcAirPct(mqRaw);
    char abuf[8];
    snprintf(abuf, sizeof(abuf), "%d%%", airPct);
    printCentered(abuf, 26, 3);
    bool alarm = false;
    display.setTextSize(2);
    display.setCursor(0, 50);
    if (airPct < 35) display.print(F("CLEAN"));
    else if (airPct < 60) display.print(F("MED"));
    else if (airPct < 80) display.print(F("DIRTY"));
    else { display.print(F("DANGER")); alarm = true; }
    display.setTextSize(1);
    display.setCursor(84, 0);
    display.print(F("V:"));
    display.print(airPct);
    display.print(F("%"));
    display.setCursor(84, 10);
    display.print(F("Alarm:"));
    display.print(alarm ? F("Y") : F("N"));
    display.setCursor(84, 20);
    display.print(F("Raw:"));
    display.print(mqRaw);
  } else {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println(F("MOTION"));
    if (hasMPU && accInitialized) {
      display.setTextSize(1);
      display.setCursor(0, 18);
      display.print(F("X: ")); display.println((int)axE);
      display.setCursor(0, 30);
      display.print(F("Y: ")); display.println((int)ayE);
      display.setCursor(0, 42);
      display.print(F("Z: ")); display.println((int)azE);
      const bool shake = isShakeActive();
      display.setTextSize(2);
      display.setCursor(20, 50);
      display.println(shake ? F("YES!") : F("NO"));
    } else {
      display.setTextSize(2);
      printCentered("NO SENSOR", 28, 2);
    }
  }
  display.display();
  delay(20);
}