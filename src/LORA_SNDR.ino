// Lilygo LORA32 Weather Station - BME280 & Weather Shield with Deep Sleep
// Board: TTGO LoRa32 V2.1 (adjust pins if using a different version)
// Sensors: BME280 (Temp/Hum/Press) + SparkFun Weather Shield (Wind/Rain)
// AS3935 removed. LoRa JSON payload aligned; includes avg wind, gust, rainfall.
// Active: 60s sampling + averaging; then LoRa send; then deep sleep 5 min.

// --- Libraries ---
#include <SPI.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <soc/rtc.h>
#include <ArduinoJson.h>
#include <math.h>

// --- Debug Configuration ---
#define DEBUG true  // true: OLED + Serial verbose; false: low-power headless

// --- Deep Sleep / Active Windows ---
#define DEEP_SLEEP_DURATION_US (5ULL * 60ULL * 1000000ULL) // 5 minutes
#define ACTIVE_SECONDS         60UL                         // 1 minute

// --- Pins (TTGO LoRa32 V2.1) ---
#define LORA_SCK   5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS   18
#define LORA_RST  14
#define LORA_DIO0 26

#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 16   // not used directly by u8g2 ctor here

#define RAIN_PIN        34  // input only
#define WIND_SPEED_PIN  35  // input only
#define WIND_DIR_PIN    36  // ADC1_CH0 / SVP

// --- I2C / Sensors ---
#define BME280_I2C_ADDR 0x77

// --- LoRa ---
#define LORA_FREQUENCY 868E6

// --- Weather / Math ---
#define RAIN_MM_PER_TIP         0.2794f   // SparkFun spec
#define WIND_KMH_PER_PULSE_HZ   2.4f      // SparkFun spec (1 Hz -> 2.4 km/h)
#define SAMPLE_INTERVAL_MS      250UL     // wind dir sample/accumulator
#define SPEED_WINDOW_MS         1000UL    // 1-second gust window
#define DEBOUNCE_DELAY_US       10000UL   // 10 ms debounce for anemometer

// --- Helpers ---
#define PULSES_TO_KMH(pulses, seconds) ( ((float)(pulses) / (float)(seconds)) * WIND_KMH_PER_PULSE_HZ )

// --- Objects ---
#if DEBUG
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA);
#endif
Adafruit_BME280 bme;

// --- Globals: Environment ---
float temperature   = NAN;
float humidity      = NAN;
float pressure_hPa  = NAN;

// --- ISR counters (volatile) ---
volatile unsigned long isr_rainClicks = 0;
volatile unsigned long isr_windClicks = 0;
volatile unsigned long lastWindPulseTime = 0;

// --- Minute accumulators ---
unsigned long minuteWindPulses = 0;
unsigned long minuteRainClicks = 0;
float        minuteGustKmh     = 0.0f;

// --- Direction vector averaging ---
double dirSinSum  = 0.0;
double dirCosSum  = 0.0;
unsigned long dirSamples = 0;
int    windDirADC = 0;
String windDirectionStr = "---";   // human-friendly (8-point)

// --- Runtime timing ---
unsigned long activePhaseStartTime = 0;
unsigned long activeEndMs          = 0;
unsigned long secWindowStart       = 0;
unsigned long secWindowPulses      = 0;
unsigned long nextSampleMs         = 0;

// --- Derived display/tx fields (for compatibility with your original send/display) ---
float windSpeedKmh   = 0.0f;  // minute average km/h
float totalRainfallMm = 0.0f; // minute total

// --- ISRs ---
void IRAM_ATTR rain_isr() { isr_rainClicks++; }

void IRAM_ATTR wind_speed_isr() {
  unsigned long now = micros();
  if (now - lastWindPulseTime > DEBOUNCE_DELAY_US) {
    isr_windClicks++;
    lastWindPulseTime = now;
  }
}

// --- Forward declarations ---
void readEnvSensors();
String getWindDirection(int adcValue);
double adcToRadians(int adc);
String angleToCardinal(double a);
void displayInfo();
void sendLoRaPacket();
void cleanPowerDown();

void setup() {
  Serial.begin(115200);
  delay(50);

  Serial.println();
  Serial.println(F("-------------------------------------"));
  Serial.println(F("LoRa32 Weather Station - Start"));
  #if DEBUG
  Serial.println(F("DEBUG: ON"));
  #else
  Serial.println(F("DEBUG: OFF"));
  #endif
  Serial.println(F("-------------------------------------"));

  // I2C
  Wire.begin(OLED_SDA, OLED_SCL);

  // OLED (DEBUG)
  #if DEBUG
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Waking up...");
    u8g2.sendBuffer();
  #endif

  // BME280
  if (!bme.begin(BME280_I2C_ADDR, &Wire)) {
    Serial.println(F("BME280 not found!"));
  } else {
    Serial.println(F("BME280 OK"));
  }

  // ADC config for wind vane
  analogSetAttenuation(ADC_11db);
  pinMode(WIND_DIR_PIN, INPUT);

  // Rain / Wind pulses
  pinMode(RAIN_PIN, INPUT_PULLUP);
  pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
  detachInterrupt(digitalPinToInterrupt(RAIN_PIN));
  detachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN));
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN),       rain_isr,       FALLING);
  attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), wind_speed_isr, FALLING);

  // LoRa
  Serial.println(F("Initializing LoRa..."));
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println(F("LoRa init FAILED -> sleep & retry"));
    #if DEBUG
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0, 10, "LoRa FAIL, sleep");
      u8g2.sendBuffer();
    #endif
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION_US);
    esp_deep_sleep_start();
  }
  Serial.print(F("LoRa OK @ ")); Serial.println((unsigned long)LORA_FREQUENCY);

  // Active phase timing
  activePhaseStartTime = millis();
  activeEndMs          = activePhaseStartTime + (ACTIVE_SECONDS * 1000UL);
  secWindowStart       = activePhaseStartTime;
  secWindowPulses      = 0;
  nextSampleMs         = activePhaseStartTime; // sample immediately

  // Reset accumulators
  isr_rainClicks = 0;
  isr_windClicks = 0;
  minuteWindPulses = 0;
  minuteRainClicks = 0;
  minuteGustKmh    = 0.0f;
  dirSinSum = 0.0; dirCosSum = 0.0; dirSamples = 0;

  // First direction preview
  windDirADC = analogRead(WIND_DIR_PIN);

  #if DEBUG
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Active 60s...");
    u8g2.sendBuffer();
  #endif
}

void loop() {
  const unsigned long now = millis();

  // Stop condition: exactly 60 s of activity
  if ((long)(now - activeEndMs) >= 0) {
    // Final environment read
    readEnvSensors();

    // Close out any partial second for gust
    const unsigned long secElapsed = now - secWindowStart;
    if (secElapsed > 0) {
      float kmh = PULSES_TO_KMH(secWindowPulses, (float)secElapsed / 1000.0f);
      if (kmh > minuteGustKmh) minuteGustKmh = kmh;
    }

    // Fold in any remaining ISR counts (unlikely at this exact moment)
    noInterrupts();
    unsigned long tailRain = isr_rainClicks; isr_rainClicks = 0;
    unsigned long tailWind = isr_windClicks; isr_windClicks = 0;
    interrupts();
    minuteRainClicks += tailRain;
    minuteWindPulses += tailWind;

    totalRainfallMm = (float)minuteRainClicks * RAIN_MM_PER_TIP;

    // Minute average wind speed (pulses over 60 s)
    windSpeedKmh = PULSES_TO_KMH(minuteWindPulses, (float)ACTIVE_SECONDS);

    // Vector-averaged wind direction
    if (dirSamples > 0) {
      double meanAngle = atan2(dirSinSum / (double)dirSamples, dirCosSum / (double)dirSamples);
      if (meanAngle < 0) meanAngle += 2.0 * PI;
      windDirectionStr = angleToCardinal(meanAngle);
    } else {
      windDirectionStr = "---";
    }

    #if DEBUG
      displayInfo();
    #endif

    // Send packet (adds gust "WG")
    sendLoRaPacket();

    // Power down peripherals & sleep
    cleanPowerDown();
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION_US);
    esp_deep_sleep_start();
  }

  // ---- ACTIVE PERIOD ----

  // 1) Direction + accumulators every SAMPLE_INTERVAL_MS
  if (now >= nextSampleMs) {
    nextSampleMs = now + SAMPLE_INTERVAL_MS;

    // Direction sample -> vector sum
    int adc = analogRead(WIND_DIR_PIN);
    windDirADC = adc;
    double ang = adcToRadians(adc);     // -> radians (bucketed)
    dirSinSum += sin(ang);
    dirCosSum += cos(ang);
    dirSamples++;

    // Pull & zero ISR deltas atomically
    noInterrupts();
    unsigned long w = isr_windClicks; isr_windClicks = 0;
    unsigned long r = isr_rainClicks; isr_rainClicks = 0;
    interrupts();

    minuteWindPulses += w;
    minuteRainClicks += r;
    totalRainfallMm = (float)minuteRainClicks * RAIN_MM_PER_TIP;

    // Update current 1-second gust window
    secWindowPulses += w;
  }

  // 2) Once per second: compute instantaneous speed for gust & reset window
  if (now - secWindowStart >= SPEED_WINDOW_MS) {
    float secKmh = PULSES_TO_KMH(secWindowPulses, 1.0f);
    if (secKmh > minuteGustKmh) minuteGustKmh = secKmh;
    secWindowStart = now;
    secWindowPulses = 0;
  }

  // 3) Periodic environment read (optional, light)
  // You can read BME every few seconds if you want live display; otherwise final read is enough.
  static unsigned long nextEnv = 0;
  if (now >= nextEnv) {
    nextEnv = now + 5000UL; // 5 s
    readEnvSensors();
  }

  // 4) Display refresh (DEBUG)
  #if DEBUG
    static unsigned long nextDisp = 0;
    if (now >= nextDisp) {
      nextDisp = now + 500UL; // 2 Hz
      displayInfo();
    }
  #endif

  // Yield
  delay(1);
}

// --- Sensors ---
void readEnvSensors() {
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure();

  if (!isnan(t)) temperature = t;
  if (!isnan(h)) humidity = h;
  if (!isnan(p) && p > 30000.0f && p < 120000.0f) pressure_hPa = p / 100.0f;
}

// --- Wind Direction Mapping ---
// Your original (8-point) buckets; refine with calibration as needed.
// adcValue is 0..4095 (ESP32 12-bit)
String getWindDirection(int adcValue) {
       if (adcValue >= 3456) return "N";
  else if (adcValue >= 2900) return "NE";
  else if (adcValue >= 2528) return "E";
  else if (adcValue >= 1815) return "NW";
  else if (adcValue >= 1140) return "SE";
  else if (adcValue >= 660)  return "W";
  else if (adcValue >= 329)  return "SW";
  else if (adcValue >= 130)  return "S";
  else if (adcValue >= 47)   return "NW";
  else return "---";
}

// Convert ADC bucket to angle (radians). For best results, replace this with a 16-sector LUT.
double adcToRadians(int adc) {
  String d = getWindDirection(adc);
  if (d == "N")  return 0.0;
  if (d == "NE") return PI/4.0;
  if (d == "E")  return PI/2.0;
  if (d == "SE") return 3.0*PI/4.0;
  if (d == "S")  return PI;
  if (d == "SW") return 5.0*PI/4.0;
  if (d == "W")  return 3.0*PI/2.0;
  if (d == "NW") return 7.0*PI/4.0;
  return 0.0;
}

// Angle (radians) -> 8-point cardinal
String angleToCardinal(double a) {
  static const char* C8[] = {"N","NE","E","SE","S","SW","W","NW"};
  int idx = (int)floor(((a + (PI/8.0)) / (2.0*PI)) * 8.0) % 8;
  return String(C8[idx]);
}

// --- OLED (DEBUG) ---
#if DEBUG
void displayInfo() {
  u8g2.clearBuffer();
  char s[32];

  // First line: wind avg + gust
  snprintf(s, sizeof(s), "Wavg:%4.1f WG:%4.1f", windSpeedKmh, minuteGustKmh);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 12, s);

  // Second line: direction + ADC
  snprintf(s, sizeof(s), "WD:%s (%d)", windDirectionStr.c_str(), windDirADC);
  u8g2.drawStr(0, 28, s);

  // Third line: rain totals
  snprintf(s, sizeof(s), "R:%4.1fmm", totalRainfallMm);
  u8g2.drawStr(0, 44, s);

  // Fourth line: env (brief)
  if (!isnan(temperature) && !isnan(humidity))
    snprintf(s, sizeof(s), "T:%2.1fC H:%2.0f%%", temperature, humidity);
  else
    snprintf(s, sizeof(s), "Env: --");
  u8g2.drawStr(0, 60, s);

  u8g2.sendBuffer();
}
#endif

// --- LoRa TX (JSON) ---
void sendLoRaPacket() {
  Serial.print(F("Sending LoRa JSON... "));

  #if DEBUG
    u8g2.setDrawColor(0); u8g2.drawBox(0, u8g2.getDisplayHeight()-12, u8g2.getDisplayWidth(), 12);
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, u8g2.getDisplayHeight()-2, "Sending LoRa...");
    u8g2.sendBuffer();
  #endif

  StaticJsonDocument<240> doc;
  if (!isnan(temperature))  doc["T"]  = round(temperature   * 10) / 10.0;
  if (!isnan(humidity))     doc["H"]  = round(humidity      * 10) / 10.0;
  if (!isnan(pressure_hPa)) doc["P"]  = round(pressure_hPa  * 10) / 10.0;

  doc["WS"] = round(windSpeedKmh * 10) / 10.0;  // minute average
  doc["WG"] = round(minuteGustKmh * 10) / 10.0; // 1s peak within the minute
  doc["WD"] = windDirectionStr;                 // 8-point
  doc["R"]  = round(totalRainfallMm * 100) / 100.0;

  String payload;
  serializeJson(doc, payload);

  LoRa.beginPacket();
  LoRa.print(payload);
  bool ok = LoRa.endPacket();
  if (ok) {
    Serial.print(F("OK: ")); Serial.println(payload);
  } else {
    Serial.println(F("FAILED"));
    #if DEBUG
      u8g2.setDrawColor(0); u8g2.drawBox(0, u8g2.getDisplayHeight()-12, u8g2.getDisplayWidth(), 12);
      u8g2.setDrawColor(1);
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0, u8g2.getDisplayHeight()-2, "LoRa Send FAIL");
      u8g2.sendBuffer();
      delay(500);
    #endif
  }
}

// --- Power-down peripherals before deep sleep ---
void cleanPowerDown() {
  // Radio
  LoRa.idle();
  LoRa.sleep();
  SPI.end();

  // OLED off
  #if DEBUG
    u8g2.setPowerSave(1);
  #endif

  // I2C release (optional)
  Wire.end();

  // Detach interrupts
  detachInterrupt(digitalPinToInterrupt(RAIN_PIN));
  detachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN));

  Serial.flush();
}
