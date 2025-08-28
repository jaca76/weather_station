// Lilygo LORA32 Weather Station - BME280 & Weather Shield with Deep Sleep
// Target Board: TTGO LoRa32 V2.1 (adjust pins if using a different version)
// Sensors: BME280 (Temp/Hum/Press) + SparkFun Weather Shield (Wind/Rain)
// AS3935 Lightning Sensor functionality has been removed.
// Display works only if DEBUG flag is true.

// Required Libraries (Install via Arduino Library Manager):
// 1. "U8g2" by olikraus (for OLED display)
// 2. "LoRa" by Sandeep Mistry (for LoRa communication)
// 3. "Adafruit BME280 Library" by Adafruit
// 4. "Adafruit Unified Sensor" by Adafruit (Dependency for BME280)
// 5. "ArduinoJson" by Benoit Blanchon (for JSON formatting)

#include <SPI.h>              // LoRa communication
#include <LoRa.h>             // LoRa library
#include <U8g2lib.h>          // OLED display library
#include <Wire.h>             // I2C communication
#include <Adafruit_Sensor.h>  // Required for BME280 library
#include <Adafruit_BME280.h>  // BME280 Sensor library
#include <soc/rtc.h>          // For lowering CPU frequency (power saving) - Optional
#include <ArduinoJson.h>      // For JSON formatting
// esp_sleep.h is usually included by the ESP32 core, needed for deep sleep functions

// --- Debug Configuration ---
#define DEBUG true // Set to true to enable OLED display and additional Serial output, false to disable

// --- Deep Sleep Configuration ---
#define DEEP_SLEEP_DURATION_US (5 * 60 * 1000000ULL) // 5 minutes in microseconds (ULL for unsigned long long)
#define ACTIVE_DURATION_MS (1 * 60 * 1000UL)       // 1 minute in milliseconds (UL for unsigned long)

// --- Pin Definitions (Check for your specific LORA32 board version) ---
// LoRa Radio Pins (Common for V2.1)
#define LORA_SCK 5            // GPIO5  -- SX127x's SCK
#define LORA_MISO 19          // GPIO19 -- SX127x's MISO
#define LORA_MOSI 27          // GPIO27 -- SX127x's MOSI
#define LORA_CS 18            // GPIO18 -- SX127x's CS
#define LORA_RST 14           // GPIO14 -- SX127x's RESET
#define LORA_DIO0 26          // GPIO26 -- SX127x's IRQ(Interrupt Request)

// OLED Display Pins (Common for V2.1 with SSD1306 128x64)
#define OLED_SDA 21           // GPIO21 - I2C Data
#define OLED_SCL 22           // GPIO22 - I2C Clock
#define OLED_RST 16           // GPIO16 (Might not be needed for all boards/displays)

// Weather Shield Sensor Pins (Connect to your LORA32 - VERIFY THESE PINS!)
#define RAIN_PIN 34           // GPIO34 - Rain Gauge Tipping Bucket (Interrupt capable, Input only)
#define WIND_SPEED_PIN 35     // GPIO35 - Anemometer (Interrupt capable, Input only)
#define WIND_DIR_PIN 36       // GPIO36 (ADC1_CH0 / SVP) - Wind Vane Analog Output

// --- I2C Configuration ---
#define BME280_I2C_ADDR 0x77  // BME280 I2C address (Alternative is 0x76)

// --- LoRa Configuration ---
#define LORA_FREQUENCY 868E6  // Adjust to your region! MUST MATCH RECEIVER

// --- Weather Sensor Constants ---
#define RAIN_MM_PER_TIP 0.2794       // Standard for SparkFun Weather Meters (0.011 inches)
#define WIND_KMH_PER_PULSE_HZ 2.4    // Standard for SparkFun Weather Meters (1 pulse/sec = 2.4 km/h)
#define WEATHER_CALC_INTERVAL 5000   // Calculate weather values every 5 seconds (ms)

// --- Debouncing for wind speed ---
#define DEBOUNCE_DELAY_US 10000      // 10ms debounce delay in microseconds

// --- Global Objects ---
#if DEBUG
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA);
#endif
Adafruit_BME280 bme; // BME280 Sensor object (I2C)

// --- Variables ---
// Environmental Sensors (from BME280)
float temperature = NAN;
float humidity = NAN;
float pressure_hPa = NAN;

// Weather Shield Sensors
volatile unsigned long rainClicks = 0;      // Reset each active cycle
volatile unsigned long windSpeedClicks = 0; // Reset each active cycle
volatile unsigned long lastWindPulseTime = 0; // For debouncing
float windSpeedKmh = 0.0;
float rainfallMm = 0.0;      // Rainfall in the current WEATHER_CALC_INTERVAL
float totalRainfallMm = 0.0; // Accumulated rainfall during the current 1-min active period
int windDirADC = 0;
String windDirectionStr = "---";

unsigned long activePhaseStartTime = 0; // To track the start of the 1-minute active window
unsigned long lastWeatherCalcTime = 0;  // For periodic sensor reads during active phase

// --- Interrupt Service Routines (ISRs) ---
void IRAM_ATTR rain_isr() { 
  rainClicks++; 
}

void IRAM_ATTR wind_speed_isr() { 
  unsigned long currentTime = micros();
  if (currentTime - lastWindPulseTime > DEBOUNCE_DELAY_US) {
    windSpeedClicks++;
    lastWindPulseTime = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize, esp. after deep sleep

  Serial.println("-------------------------------------");
  Serial.println("LoRa32 Weather Station - Woke Up / Starting");
  Serial.println("Sensors: BME280, Weather Shield (AS3935 Removed)");
  Serial.println("Data Format: JSON (Receiver Aligned)");
  #if DEBUG
  Serial.println("DEBUG Mode: ENABLED (OLED Active)");
  #else
  Serial.println("DEBUG Mode: DISABLED (OLED Inactive)");
  #endif
  Serial.print("Active for: "); Serial.print(ACTIVE_DURATION_MS / 1000); Serial.println("s");
  Serial.print("Sleep for: "); Serial.print(DEEP_SLEEP_DURATION_US / 1000000 / 60); Serial.println("min");
  Serial.println("-------------------------------------");

  // Optional: Lower CPU frequency - rtc_clk_cpu_freq_set() needs to be called in setup.
  // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M); // Example: 80MHz

  // --- Initialize I2C Bus ---
  Wire.begin(OLED_SDA, OLED_SCL);
  Serial.println("I2C Initialized.");

  // --- Initialize OLED Display (Conditional) ---
  #if DEBUG
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, "Waking up...");
  u8g2.sendBuffer();
  Serial.println("OLED Initialized (DEBUG).");
  #else
  Serial.println("OLED Skipped (DEBUG disabled).");
  #endif

  // --- Initialize BME280 Sensor ---
  if (!bme.begin(BME280_I2C_ADDR, &Wire)) {
    Serial.println("Could not find BME280! Check wiring/address.");
  } else {
    Serial.println("BME280 Initialized OK");
  }

  // --- Initialize Weather Shield Pins & Interrupts ---
  Serial.println("Initializing Weather Shield Pins...");
  
  // Set ADC attenuation for wind direction pin to handle 3.3V range
  analogSetAttenuation(ADC_11db);
  pinMode(WIND_DIR_PIN, INPUT); // Wind Vane
  
  pinMode(RAIN_PIN, INPUT_PULLUP); // Rain Gauge
  detachInterrupt(digitalPinToInterrupt(RAIN_PIN)); // Detach if previously attached
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rain_isr, FALLING);
  
  pinMode(WIND_SPEED_PIN, INPUT_PULLUP); // Wind Speed
  detachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN)); // Detach if previously attached
  attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), wind_speed_isr, FALLING);
  
  Serial.println("Weather Shield Interrupts Attached.");
  
  // Test initial wind direction reading
  windDirADC = analogRead(WIND_DIR_PIN);
  Serial.print("Initial Wind Direction ADC: "); Serial.println(windDirADC);

  // --- Initialize LoRa ---
  Serial.println("Initializing LoRa...");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed! Going to sleep and retry...");
    #if DEBUG
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "LoRa Init FAIL!");
    u8g2.drawStr(0, 25, "Sleeping...");
    u8g2.sendBuffer();
    #endif
    delay(100); 
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION_US);
    esp_deep_sleep_start();
  }
  Serial.print("LoRa Initialized OK! Freq: "); Serial.println(LORA_FREQUENCY);

  #if DEBUG
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, "Active Phase...");
  u8g2.sendBuffer();
  #endif
  Serial.println("Setup Complete. Entering active phase.");

  activePhaseStartTime = millis();
  lastWeatherCalcTime = activePhaseStartTime; 
  rainClicks = 0;
  windSpeedClicks = 0;
  rainfallMm = 0.0;
  totalRainfallMm = 0.0;
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - activePhaseStartTime < ACTIVE_DURATION_MS) {
    // --- WITHIN 1-MINUTE ACTIVE WINDOW ---

    if (currentTime - lastWeatherCalcTime >= WEATHER_CALC_INTERVAL) {
      readEnvSensors(); 
      calculateWeatherShieldData(currentTime); 
      lastWeatherCalcTime = currentTime; 
    }

    #if DEBUG
    displayInfo(); 
    #endif

    delay(10); 

  } else {
    // --- 1-MINUTE ACTIVE DURATION ENDED ---
    Serial.println("Active period finished. Sending final data and preparing for sleep...");
    
    // One final read before sending to ensure data is as fresh as possible
    readEnvSensors();
    calculateWeatherShieldData(millis()); // Use current time for this final calculation
    #if DEBUG
    displayInfo(); // Show final values if DEBUG is enabled
    #endif

    sendLoRaPacket(); 

    #if DEBUG
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Data sent.");
    u8g2.drawStr(0, 25, "Going to sleep");
    u8g2.drawStr(0, 40, "for 5 minutes...");
    u8g2.sendBuffer();
    #endif
    Serial.println("Entering deep sleep for 5 minutes.");
    delay(200); 

    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION_US);
    esp_deep_sleep_start();
  }
}

// --- Function to Read BME280 Environmental Sensor ---
void readEnvSensors() {
  float newTemp = bme.readTemperature(); 
  float newHum = bme.readHumidity();     
  float newPress = bme.readPressure();   

  if (!isnan(newTemp)) { temperature = newTemp; } 
  else { Serial.println("Failed to read temperature from BME280"); }

  if (!isnan(newHum)) { humidity = newHum; } 
  else { Serial.println("Failed to read humidity from BME280"); }

  if (!isnan(newPress) && newPress > 30000.0 && newPress < 120000.0) { 
      pressure_hPa = newPress / 100.0F; 
  } else {
      Serial.print("Failed or implausible pressure reading from BME280: "); Serial.println(newPress);
  }
}

// --- Function to Calculate Weather Shield Data ---
void calculateWeatherShieldData(unsigned long currentTime) {
  unsigned long currentWindClicks;
  noInterrupts(); 
  currentWindClicks = windSpeedClicks;
  windSpeedClicks = 0; 
  interrupts(); 

  float intervalSeconds = (currentTime - lastWeatherCalcTime) / 1000.0; 
  if (intervalSeconds <= 0) intervalSeconds = (float)WEATHER_CALC_INTERVAL / 1000.0; 

  float windPulseHz = (float)currentWindClicks / intervalSeconds;
  windSpeedKmh = windPulseHz * WIND_KMH_PER_PULSE_HZ;

  unsigned long currentRainClicks;
  noInterrupts(); 
  currentRainClicks = rainClicks;
  rainClicks = 0; 
  interrupts(); 

  rainfallMm = (float)currentRainClicks * RAIN_MM_PER_TIP; 
  totalRainfallMm += rainfallMm; 

  // Take multiple readings of wind direction and average them
  int windDirSum = 0;
  const int numReadings = 5;
  for (int i = 0; i < numReadings; i++) {
    windDirSum += analogRead(WIND_DIR_PIN);
    delay(10); // Small delay between readings
  }
  windDirADC = windDirSum / numReadings;
  windDirectionStr = getWindDirection(windDirADC);

  Serial.print(currentTime / 1000); Serial.print("s (active) - ");
  Serial.print("T:"); Serial.print(temperature, 1); Serial.print("C, ");
  Serial.print("H:"); Serial.print(humidity, 0); Serial.print("%, ");
  Serial.print("P:"); Serial.print(pressure_hPa, 1); Serial.print("hPa | ");
  Serial.print("WS:"); Serial.print(windSpeedKmh, 1); Serial.print("km/h("); Serial.print(currentWindClicks); Serial.print("c), ");
  Serial.print("WD:"); Serial.print(windDirectionStr); Serial.print("("); Serial.print(windDirADC); Serial.print("), ");
  Serial.print("Rnow:"); Serial.print(rainfallMm, 2); Serial.print("mm, Rtotal:"); Serial.print(totalRainfallMm, 2); Serial.print("mm("); Serial.print(currentRainClicks); Serial.println("c)");
}

// --- Function to Convert Wind Vane ADC Reading to Direction ---
// Updated thresholds for ESP32 12-bit ADC (0-4095 range)
String getWindDirection(int adcValue) {
  // These values are calibrated for a 3.3V supply and a 9.3kΩ pull-down resistor.
  // Using 8 directions (instead of 16) for more stable readings due to ADC fluctuations.

       if (adcValue >= 3456) return "N";    // Boundary: NNE
  else if (adcValue >= 2900) return "NE";   // Boundary: ENE     
  else if (adcValue >= 2528) return "E";    // Boundary: ESE
  else if (adcValue >= 1815) return "NW";   // Boundary: ENE
  else if (adcValue >= 1140) return "SE";   // Boundary: SSE
  else if (adcValue >= 660)  return "W";    // Boundary: SSW
  else if (adcValue >= 329)  return "SW";   // Boundary: WSW
  else if (adcValue >= 130)  return "S";    // Boundary: WNW
  else return "---"; 
}


// --- Function to Display Info on OLED (Conditional) ---
#if DEBUG
void displayInfo() {
  u8g2.clearBuffer();
  char dataStr[32]; 

  u8g2.setFont(u8g2_font_ncenB08_tr);
  // if (!isnan(temperature)) snprintf(dataStr, sizeof(dataStr), "T:%.1fC", temperature); else snprintf(dataStr, sizeof(dataStr), "T:---C");
  // u8g2.drawStr(0, 10, dataStr);
  // if (!isnan(humidity)) snprintf(dataStr, sizeof(dataStr), "H:%.0f%%", humidity); else snprintf(dataStr, sizeof(dataStr), "H:--%%");
  // u8g2.drawStr(64, 10, dataStr); 
  // if (!isnan(pressure_hPa)) snprintf(dataStr, sizeof(dataStr), "P:%.0fhPa", pressure_hPa); else snprintf(dataStr, sizeof(dataStr), "P:----hPa"); 
  // u8g2.drawStr(0, 25, dataStr);

  // u8g2.drawStr(0, 40, "No Lightning Sensor"); // Placeholder or remove
  // The following lines are related to wind direction and screen, so they are not commented out.
  snprintf(dataStr, sizeof(dataStr), "W:%s %.1fkph", windDirectionStr.c_str(), windSpeedKmh);
  u8g2.drawStr(0, 55, dataStr);
  // The following lines are related to wind direction and screen, so they are not commented out.
  int windStrLen = u8g2.getStrWidth(dataStr);
  snprintf(dataStr, sizeof(dataStr), "R:%.1f/%.1fmm", rainfallMm, totalRainfallMm);
  if (windStrLen < 70) { 
    u8g2.drawStr(windStrLen + 5, 55, dataStr);
  } else { 
    u8g2.drawStr(80, 55, dataStr); 
  }
  
  u8g2.sendBuffer();
}
#endif

// --- Function to Send LoRa Packet (JSON - RECEIVER ALIGNED) ---
void sendLoRaPacket() {
  Serial.print("Sending LoRa JSON packet (Receiver Aligned)... ");

  #if DEBUG
  u8g2.setDrawColor(0); u8g2.drawBox(0, u8g2.getDisplayHeight() - 12, u8g2.getDisplayWidth(), 12); 
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_ncenB08_tr); 
  u8g2.drawStr(0, u8g2.getDisplayHeight() -2 , "Sending LoRa..."); 
  u8g2.sendBuffer();
  #endif

  StaticJsonDocument<200> jsonDoc; // Reduced size as lightning data is removed

  // Assign numeric types directly to JSON doc for better performance and memory usage.
  // The round() trick is an efficient way to control decimal precision.
  if (!isnan(temperature)) jsonDoc["T"] = round(temperature * 10) / 10.0;
  if (!isnan(humidity)) jsonDoc["H"] = round(humidity * 10) / 10.0;
  if (!isnan(pressure_hPa)) jsonDoc["P"] = round(pressure_hPa * 10) / 10.0;

  jsonDoc["WS"] = round(windSpeedKmh * 10) / 10.0;
  jsonDoc["WD"] = windDirectionStr;
  jsonDoc["R"] = round(totalRainfallMm * 100) / 100.0;

  String jsonPacket;
  // serializeJson will convert the numbers to text in the JSON string.
  serializeJson(jsonDoc, jsonPacket);

  LoRa.beginPacket();
  LoRa.print(jsonPacket);
  if(LoRa.endPacket()) {
    Serial.print("Sent OK: ");
    Serial.println(jsonPacket);
  } else {
    Serial.println("Send FAILED");
    #if DEBUG
    u8g2.setDrawColor(0); u8g2.drawBox(0, u8g2.getDisplayHeight() - 12, u8g2.getDisplayWidth(), 12);
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, u8g2.getDisplayHeight() - 2, "LoRa Send FAIL");
    u8g2.sendBuffer();
    delay(1000); 
    #endif
  }
}
