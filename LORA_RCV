// LoRa to MQTT Receiver for Home Assistant with OLED Display, HA Discovery, and AS3935 (SparkFun Library) - Corrected
// Target Board: TTGO LoRa32 V2.1

// Required Libraries (Install via Arduino Library Manager):
// 1. "LoRa" by Sandeep Mistry
// 2. "ArduinoJson" by Benoit Blanchon (Version 6.x or newer recommended)
// 3. "PubSubClient" by Nick O'Leary (ensure version 2.8.0 or newer for setBufferSize)
// 4. "U8g2" by olikraus (for OLED display)
// 5. "SparkFun AS3935 Lightning Detector" by SparkFun Electronics
// 6. (WiFi is built-in for ESP32)

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>      // OLED display library
#include <Wire.h>         // Required for I2C communication (OLED, AS3935)
#include <SparkFun_AS3935.h> // SparkFun AS3935 Lightning Detector Library

// --- Software Version ---
const char* SW_VERSION = "1.3.1_sparkfun_as3935_fixes"; // Updated version

// --- WiFi Configuration ---
const char* WIFI_SSID = "GL-MT6000-da9";
const char* WIFI_PASSWORD = "T5R4DZECBW";

// --- MQTT Broker Configuration ---
const char* MQTT_BROKER_ADDRESS = "192.168.8.200";
const int MQTT_BROKER_PORT = 1883;
const char* MQTT_CLIENT_ID = "loraWeatherReceiver";
const char* MQTT_USERNAME = "jacek";
const char* MQTT_PASSWORD = "Jaca19&^";

// --- Home Assistant Device Info ---
const char* HA_DEVICE_NAME = "LoRa Weather Station Receiver";
const char* HA_MANUFACTURER = "DIY";
const char* HA_MODEL = "TTGO LoRa32 V2.1";

// --- MQTT State Topics ---
const char* MQTT_TOPIC_TEMPERATURE = "homeassistant/sensor/lora_weather_station/temperature/state";
const char* MQTT_TOPIC_HUMIDITY = "homeassistant/sensor/lora_weather_station/humidity/state";
const char* MQTT_TOPIC_PRESSURE = "homeassistant/sensor/lora_weather_station/pressure/state";
const char* MQTT_TOPIC_WIND_SPEED = "homeassistant/sensor/lora_weather_station/wind_speed/state";
const char* MQTT_TOPIC_WIND_DIRECTION = "homeassistant/sensor/lora_weather_station/wind_direction/state";
const char* MQTT_TOPIC_RAINFALL = "homeassistant/sensor/lora_weather_station/rainfall/state";
// AS3935 Specific Topics
const char* MQTT_TOPIC_AS3935_LIGHTNING_DISTANCE = "homeassistant/sensor/lora_weather_station/as3935_lightning_distance/state";
const char* MQTT_TOPIC_AS3935_EVENT = "homeassistant/sensor/lora_weather_station/as3935_event/state";
// LoRa Specific
const char* MQTT_TOPIC_LAST_PACKET_RSSI = "homeassistant/sensor/lora_weather_station/rssi/state";
// General
const char* MQTT_TOPIC_CONNECTIVITY_STATE = "homeassistant/binary_sensor/lora_weather_station/connectivity/state";

// --- MQTT Configuration Topics (for Home Assistant Discovery) ---
const char* HA_DISCOVERY_PREFIX = "homeassistant";
const char* MQTT_CONFIG_TOPIC_TEMPERATURE = "homeassistant/sensor/lora_weather_station_temperature/config";
const char* MQTT_CONFIG_TOPIC_HUMIDITY = "homeassistant/sensor/lora_weather_station_humidity/config";
const char* MQTT_CONFIG_TOPIC_PRESSURE = "homeassistant/sensor/lora_weather_station_pressure/config";
const char* MQTT_CONFIG_TOPIC_WIND_SPEED = "homeassistant/sensor/lora_weather_station_wind_speed/config";
const char* MQTT_CONFIG_TOPIC_WIND_DIRECTION = "homeassistant/sensor/lora_weather_station_wind_direction/config";
const char* MQTT_CONFIG_TOPIC_RAINFALL = "homeassistant/sensor/lora_weather_station_rainfall/config";
// AS3935 Config Topics
const char* MQTT_CONFIG_TOPIC_AS3935_LIGHTNING_DISTANCE = "homeassistant/sensor/lora_weather_station_as3935_lightning_distance/config";
const char* MQTT_CONFIG_TOPIC_AS3935_EVENT = "homeassistant/sensor/lora_weather_station_as3935_event/config";
// LoRa Config Topic
const char* MQTT_CONFIG_TOPIC_RSSI = "homeassistant/sensor/lora_weather_station_rssi/config";
// General Config Topic
const char* MQTT_CONFIG_TOPIC_CONNECTIVITY = "homeassistant/binary_sensor/lora_weather_station_connectivity/config";

// --- LoRa Radio Pin Definitions (TTGO LoRa32 V2.1 specific) ---
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_DIO0 26

// --- OLED Display Pin Definitions (TTGO LoRa32 V2.1 specific) ---
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 16

// --- AS3935 Lightning Sensor (SparkFun Library) Pins & Config ---
#define AS3935_IRQ_PIN 13
#define AS3935_I2C_ADDR 0x03
#define AS3935_INT_L 0x08
// --- LoRa Configuration ---
#define LORA_FREQUENCY 868E6
#define ANTFREQ 3
// --- JSON Configuration ---
const int JSON_DOC_SIZE_LORA = 256;
const int JSON_DOC_SIZE_HA_CONFIG = 768;
const int LORA_BUFFER_SIZE = 256;
const int MQTT_PUBLISH_BUFFER_SIZE = 1024;

// --- Global Objects ---
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA);
SparkFun_AS3935 lightning(AS3935_I2C_ADDR);

// --- Display Status Variables ---
String displayWifiStatus = "WiFi: Init...";
String displayMqttStatus = "MQTT: Init...";
String displayLoRaStatus = "LoRa: Init...";
String displayAs3935Status = "AS3935: Init...";
String displayLastTemp = "T: --- C";
String displayLastHum = "H: --- %";
unsigned long lastDisplayUpdateTime = 0;
const unsigned long displayUpdateInterval = 1000;
unsigned long lastLoRaPacketMillis = 0;
unsigned long lastAs3935ActivityMillis = 0;

// --- LoRa Packet Handling ---
volatile bool newLoRaPacketReceived = false;
char loraPacketBuffer[LORA_BUFFER_SIZE];
volatile int lastPacketRssi = 0;

// --- AS3935 Interrupt Handling ---
volatile bool as3935InterruptFlag = false;
int AS3935_TUNING_CAP_VALUE=10;
// --- Home Assistant Discovery ---
bool ha_discovery_published = false;

void IRAM_ATTR handleAS3935Interrupt() {
  as3935InterruptFlag = true;
}

void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, displayWifiStatus.c_str());
  u8g2.drawStr(0, 22, displayMqttStatus.c_str());
  u8g2.drawStr(0, 34, displayLoRaStatus.c_str());
  u8g2.drawStr(0, 46, displayAs3935Status.c_str());
  String tempHumStr = displayLastTemp + " " + displayLastHum;
  u8g2.drawStr(0, 58, tempHumStr.c_str());
  u8g2.sendBuffer();
}

void showDisplayMessage(const char* msg1, const char* msg2 = "", const char* msg3 = "", int delay_ms = 2000) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if (strlen(msg1) > 0) u8g2.drawStr(0, 20, msg1);
    if (strlen(msg2) > 0) u8g2.drawStr(0, 35, msg2);
    if (strlen(msg3) > 0) u8g2.drawStr(0, 50, msg3);
    u8g2.sendBuffer();
    if (delay_ms > 0) delay(delay_ms);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print(F("Connecting to WiFi SSID: ")); 
  Serial.println(WIFI_SSID);
  displayWifiStatus = "WiFi: Connecting";
  updateDisplay();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); 
    Serial.print(".");
    retries++;
    if (retries % 2 == 0) displayWifiStatus = "WiFi: Connecting.";
    else displayWifiStatus = "WiFi: Connecting..";
    updateDisplay();
    if (retries > 30) {
        Serial.println(F("\nFailed to connect to WiFi. Restarting..."));
        displayWifiStatus = "WiFi: FAILED";
        showDisplayMessage("WiFi Connect Failed", "Check Credentials", "Restarting...", 3000);
        ESP.restart();
    }
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println(F("WiFi connected"));
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
  displayWifiStatus = "WiFi: " + WiFi.localIP().toString();
  updateDisplay();
}

int getBestTune()
{
  char dataStr[32]; 
  int tempTune=0;
	int target = 3125, currentcount = 0, bestdiff = INT_MAX, currdiff = 0;
	byte bestTune = 0, currTune = 0;
	unsigned long setUpTime;
	int currIrq, prevIrq;
	// set lco_fdiv divider to 0, which translates to 16
	// so we are looking for 31250Hz on irq pin (square wave)
	// and since we are counting for 100ms that translates to number 3125
	// each capacitor changes second least significant digit
	// using this timing so this is probably the best way to go
	//registerWrite(AS3935_LCO_FDIV,0);
	lightning.changeDivRatio(0);
  //registerWrite(AS3935_DISP_LCO,1);
  lightning.displayOscillator(true, ANTFREQ);
	// tuning is not linear, can't do any shortcuts here
	// going over all built-in cap values and finding the best
	for (currTune = 0; currTune <= 0x0F; currTune++) 
	{
    tempTune=currTune*15;
		lightning.tuneCap(tempTune);
		// let it settle
		delay(10);
		currentcount = 0;
		prevIrq = digitalRead(AS3935_IRQ_PIN);
		setUpTime = millis() + 100;
		while((long)(millis() - setUpTime) < 0)
		{
			currIrq = digitalRead(AS3935_IRQ_PIN);
			if (currIrq > prevIrq)
			{
				currentcount++;	
			}
			prevIrq = currIrq;
		}
		currdiff = target - currentcount;
		// don't look at me, abs() misbehaves
		if(currdiff < 0)
			currdiff = -currdiff;
		if(bestdiff > currdiff)
		{
			bestdiff = currdiff;
			bestTune = currTune;
		}
	}
  tempTune=bestTune*15;
	lightning.tuneCap(tempTune);
  Serial.print(F("BestTune: "));
  Serial.println(tempTune);
  snprintf(dataStr, sizeof(dataStr), "%.1f", tempTune);
  showDisplayMessage("BestTune", "koniec:", dataStr, 1000); 
	delay(2000);
	lightning.displayOscillator(false, ANTFREQ);
	// and now do RCO calibration
	setup_as3935();
	// if error is over 109, we are outside allowed tuning range of +/-3.5%
	delay(2000);
  return bestTune;
}


void setup_as3935() {
  Serial.println(F("Initializing AS3935 Lightning Sensor (SparkFun)..."));
  displayAs3935Status = "AS3935: Init...";
  updateDisplay();
  if (!lightning.begin(Wire)) {
    Serial.println (F("AS3935 Lightning Detector not found. Check wiring and I2C address."));
    displayAs3935Status = "AS3935: FAILED!";
    updateDisplay();
    return;
  }
  Serial.println(F("AS3935 Lightning Detector found."));
  // Use enum from SparkFun_AS3935.h: typedef enum { INDOOR=0x12, OUTDOOR=0x0E, DEFAULT_INDOOR=0x24 } settings_t;
  lightning.setIndoorOutdoor(INDOOR); // Corrected constant
  Serial.println(F("AS3935 set to Indoor mode."));
  lightning.setNoiseLevel(2);
  lightning.watchdogThreshold(2);// Default 1
  lightning.spikeRejection(4); // Deafault 2
  lightning.maskDisturber(true);
  lightning.tuneCap(5);
  pinMode(AS3935_IRQ_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(AS3935_IRQ_PIN), handleAS3935Interrupt, RISING);
  Serial.println(F("AS3935 interrupt attached."));
  displayAs3935Status = "AS3935: Listening";
  lastAs3935ActivityMillis = millis();
  updateDisplay();
  delay(1000);
}

void processAS3935Data() {
  if (as3935InterruptFlag) {
    as3935InterruptFlag = false;
    delay(5);
    // Use enum from SparkFun_AS3935.h: typedef enum { NH=1, DIST=4, LIGHT=8 } interrupt_t;
    uint8_t intVal = lightning.readInterruptReg();
    String eventTypeStr = "Unknown";
    int distanceKm = -1;
    lastAs3935ActivityMillis = millis();

    if (intVal == NOISE_TO_HIGH ) { // Corrected constant
      eventTypeStr = "Noise";
      Serial.println(F("AS3935: Noise event detected."));
      displayAs3935Status = "AS3935: Noise";
    } else if (intVal == DISTURBER_DETECT ) { // Corrected constant
      eventTypeStr = "Disturber";
      Serial.println(F("AS3935: Disturber detected."));
      displayAs3935Status = "AS3935: Disturber";
    } else if (intVal == LIGHTNING) { // Corrected constant
      eventTypeStr = "Lightning";
      distanceKm = lightning.distanceToStorm();
      Serial.print(F("AS3935: Lightning detected! Distance: "));
      if (distanceKm == 1) {
        Serial.print(F("Overhead")); 
        displayAs3935Status = "AS3935: Light OH";
      } else if (distanceKm == 63) {
        Serial.print(F("Out of range")); 
        displayAs3935Status = "AS3935: Light OOR";
      } else { 
        Serial.print(distanceKm); Serial.print(F("km")); 
        displayAs3935Status = "AS3935: Light " + String(distanceKm) + "km";
      }
      Serial.println();
      if (mqttClient.connected()) {
        if (distanceKm == 63) {
           mqttClient.publish(MQTT_TOPIC_AS3935_LIGHTNING_DISTANCE, "clear", true);
        } else {
           mqttClient.publish(MQTT_TOPIC_AS3935_LIGHTNING_DISTANCE, String(distanceKm).c_str(), true);
        }
      }
    } else {
        Serial.print(F("AS3935: Unknown interrupt value: 0x")); Serial.println(intVal, HEX);
        displayAs3935Status = "AS3935: Unk Int";
    }
    if (mqttClient.connected()) {
        mqttClient.publish(MQTT_TOPIC_AS3935_EVENT, eventTypeStr.c_str(), true);
    }
    updateDisplay();
  }
}

void publishSingleSensorDiscovery(const char* component, 
                                  const char* sensor_id_suffix, 
                                  const char* config_topic,
                                  const char* state_topic,
                                  const char* name,
                                  const char* device_class, 
                                  const char* unit_of_measurement, 
                                  const char* icon, 
                                  JsonObject& device_payload, 
                                  const char* availability_topic,
                                  const char* payload_on = nullptr, 
                                  const char* payload_off = nullptr 
                                  ) {
    StaticJsonDocument<JSON_DOC_SIZE_HA_CONFIG> configDoc; 
    configDoc["name"] = name;
    String unique_id = String(MQTT_CLIENT_ID) + "_" + sensor_id_suffix;
    configDoc["unique_id"] = unique_id;
    configDoc["state_topic"] = state_topic;
    if (device_class && strlen(device_class) > 0) configDoc["device_class"] = device_class;
    if (unit_of_measurement && strlen(unit_of_measurement) > 0) configDoc["unit_of_measurement"] = unit_of_measurement;
    if (icon && strlen(icon) > 0) configDoc["icon"] = icon;
    if (payload_on && strlen(payload_on) > 0) configDoc["payload_on"] = payload_on;
    if (payload_off && strlen(payload_off) > 0) configDoc["payload_off"] = payload_off;
    configDoc["availability_topic"] = availability_topic;
    configDoc["payload_available"] = "online";
    configDoc["payload_not_available"] = "offline";
    configDoc["device"] = device_payload; 
    String output;
    serializeJson(configDoc, output);
    Serial.print(F("Publikowanie konfiguracji HA dla '")); Serial.print(name); 
    Serial.print(F("' do tematu: ")); Serial.println(config_topic);
    Serial.println(F("Ładunek JSON konfiguracyjny:"));
    Serial.println(output);
    Serial.println("Rozmiar ładunku konfiguracyjnego JSON: " + String(output.length()) + " bajtów");
    if (mqttClient.publish(config_topic, output.c_str(), true)) { 
        Serial.println(F("  Pomyślnie opublikowano konfigurację."));
    } else {
        Serial.println(F("  BŁĄD: Nie udało się opublikować konfiguracji. Sprawdź rozmiar bufora MQTT i połączenie."));
    }
    delay(100); 
}

void publishHomeAssistantDiscoveryMessages() {
    Serial.println(F("Publikowanie komunikatów Home Assistant Discovery..."));
    StaticJsonDocument<384> deviceDoc; 
    JsonObject device_payload = deviceDoc.to<JsonObject>();
    device_payload["identifiers"] = MQTT_CLIENT_ID; 
    device_payload["name"] = HA_DEVICE_NAME;
    device_payload["model"] = HA_MODEL;
    device_payload["manufacturer"] = HA_MANUFACTURER;
    device_payload["sw_version"] = SW_VERSION;
    publishSingleSensorDiscovery("sensor", "temperature", MQTT_CONFIG_TOPIC_TEMPERATURE, MQTT_TOPIC_TEMPERATURE, "LoRa Temperatura", "temperature", "°C", nullptr, device_payload, MQTT_TOPIC_CONNECTIVITY_STATE);
    publishSingleSensorDiscovery("sensor", "humidity", MQTT_CONFIG_TOPIC_HUMIDITY, MQTT_TOPIC_HUMIDITY, "LoRa Wilgotność", "humidity", "%", nullptr, device_payload, MQTT_TOPIC_CONNECTIVITY_STATE);
    publishSingleSensorDiscovery("sensor", "pressure", MQTT_CONFIG_TOPIC_PRESSURE, MQTT_TOPIC_PRESSURE, "LoRa Ciśnienie", "pressure", "hPa", nullptr, device_payload, MQTT_TOPIC_CONNECTIVITY_STATE);
    publishSingleSensorDiscovery("sensor", "wind_speed", MQTT_CONFIG_TOPIC_WIND_SPEED, MQTT_TOPIC_WIND_SPEED, "LoRa Prędkość wiatru", "wind_speed", "m/s", "mdi:weather-windy", device_payload, MQTT_TOPIC_CONNECTIVITY_STATE);
    publishSingleSensorDiscovery("sensor", "wind_direction", MQTT_CONFIG_TOPIC_WIND_DIRECTION, MQTT_TOPIC_WIND_DIRECTION, "LoRa Kierunek wiatru", nullptr, nullptr, "mdi:compass-outline", device_payload, MQTT_TOPIC_CONNECTIVITY_STATE);
    publishSingleSensorDiscovery("sensor", "rainfall", MQTT_CONFIG_TOPIC_RAINFALL, MQTT_TOPIC_RAINFALL, "LoRa Opady deszczu", "precipitation", "mm", "mdi:weather-pouring", device_payload, MQTT_TOPIC_CONNECTIVITY_STATE);
    publishSingleSensorDiscovery("sensor", "rssi", MQTT_CONFIG_TOPIC_RSSI, MQTT_TOPIC_LAST_PACKET_RSSI, "LoRa RSSI", "signal_strength", "dBm", nullptr, device_payload, MQTT_TOPIC_CONNECTIVITY_STATE);
    publishSingleSensorDiscovery("sensor", "as3935_lightning_distance", MQTT_CONFIG_TOPIC_AS3935_LIGHTNING_DISTANCE, MQTT_TOPIC_AS3935_LIGHTNING_DISTANCE, "AS3935 Dystans błyskawicy", "distance", "km", "mdi:weather-lightning", device_payload, MQTT_TOPIC_CONNECTIVITY_STATE);
    publishSingleSensorDiscovery("sensor", "as3935_event", MQTT_CONFIG_TOPIC_AS3935_EVENT, MQTT_TOPIC_AS3935_EVENT, "AS3935 Zdarzenie", nullptr, nullptr, "mdi:flash-alert", device_payload, MQTT_TOPIC_CONNECTIVITY_STATE);
    publishSingleSensorDiscovery("binary_sensor", "connectivity", MQTT_CONFIG_TOPIC_CONNECTIVITY, MQTT_TOPIC_CONNECTIVITY_STATE, "LoRa Odbiornik Połączenie", "connectivity", nullptr, nullptr, device_payload, MQTT_TOPIC_CONNECTIVITY_STATE, "online", "offline");
    Serial.println(F("Zakończono publikowanie komunikatów Home Assistant Discovery."));
}

void reconnect_mqtt() {
  int retries = 0;
  while (!mqttClient.connected()) {
    Serial.print(F("Próba połączenia z MQTT... "));
    displayMqttStatus = "MQTT: Connecting";
    updateDisplay();
    bool connected;
    String lwt_topic = MQTT_TOPIC_CONNECTIVITY_STATE;
    const char* lwt_message = "offline";
    uint8_t lwt_qos = 0;       
    boolean lwt_retain = true; 
    if (strlen(MQTT_USERNAME) > 0 && strlen(MQTT_PASSWORD) > 0) {
        connected = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, lwt_topic.c_str(), lwt_qos, lwt_retain, lwt_message);
    } else {
        connected = mqttClient.connect(MQTT_CLIENT_ID, lwt_topic.c_str(), lwt_qos, lwt_retain, lwt_message);
    }
    if (connected) {
      Serial.println(F("połączono."));
      displayMqttStatus = "MQTT: Connected";
      updateDisplay();
      mqttClient.publish(MQTT_TOPIC_CONNECTIVITY_STATE, "online", true); 
      if (!ha_discovery_published) {
        publishHomeAssistantDiscoveryMessages();
        ha_discovery_published = true; 
      }
    } else {
      Serial.print(F("niepowodzenie, rc="));
      Serial.print(mqttClient.state());
      Serial.println(F(" spróbuj ponownie za 1 sekundę"));
      displayMqttStatus = "MQTT: Fail " + String(mqttClient.state());
      updateDisplay();
      retries++;
      if (retries > 5) {
          Serial.println(F("BŁĄD: Połączenie MQTT nie powiodło się po wielu próbach."));
          displayMqttStatus = "MQTT: FAILED";
          updateDisplay();
          delay(1000); 
          return;
      }
      delay(1000); 
    }
  }
}

void IRAM_ATTR onLoRaReceive_ISR(int packetSize) {
  if (packetSize == 0 || newLoRaPacketReceived) return; 
  int len = 0;
  while (LoRa.available() && len < LORA_BUFFER_SIZE -1) {
    loraPacketBuffer[len++] = (char)LoRa.read();
  }
  loraPacketBuffer[len] = '\0'; 
  lastPacketRssi = LoRa.packetRssi(); 
  newLoRaPacketReceived = true;       
}

void processReceivedLoRaData() {
  lastLoRaPacketMillis = millis(); 
  displayLoRaStatus = "LoRa: RSSI " + String(lastPacketRssi) + "dBm"; 
  char localBuffer[LORA_BUFFER_SIZE];
  strncpy(localBuffer, loraPacketBuffer, LORA_BUFFER_SIZE);
  localBuffer[LORA_BUFFER_SIZE - 1] = '\0'; 
  Serial.print(F("Odebrano JSON (LoRa): "));
  Serial.println(localBuffer);
  if (mqttClient.connected()) {
      mqttClient.publish(MQTT_TOPIC_LAST_PACKET_RSSI, String(lastPacketRssi).c_str(), true); 
  }
  StaticJsonDocument<JSON_DOC_SIZE_LORA> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, localBuffer);
  if (error) {
    Serial.print(F("BŁĄD: deserializeJson() (LoRa) nie powiodło się: "));
    Serial.println(error.c_str());
    displayLoRaStatus = "LoRa: JSON Err";
    updateDisplay();
    newLoRaPacketReceived = false; 
    return; 
  }
  if (!mqttClient.connected()) {
      Serial.println(F("MQTT nie jest połączony. Pomijanie publikacji danych sensorów LoRa."));
      displayMqttStatus = "MQTT: Disconnected"; 
      updateDisplay();
      newLoRaPacketReceived = false; 
      return; 
  }
  if (jsonDoc.containsKey("T")) {
    float temp = jsonDoc["T"];
    mqttClient.publish(MQTT_TOPIC_TEMPERATURE, String(temp, 1).c_str(), true);
    Serial.print(F("Opublikowano Temp: ")); Serial.println(temp,1);
    displayLastTemp = "T: " + String(temp,1) + "C";
  }
  if (jsonDoc.containsKey("H")) {
    float hum = jsonDoc["H"];
    mqttClient.publish(MQTT_TOPIC_HUMIDITY, String(hum, 1).c_str(), true);
    Serial.print(F("Opublikowano Wilg: ")); Serial.println(hum,1);
    displayLastHum = "H: " + String(hum,1) + "%";
  }
  if (jsonDoc.containsKey("P")) {
    float press = jsonDoc["P"];
    mqttClient.publish(MQTT_TOPIC_PRESSURE, String(press, 1).c_str(), true);
    Serial.print(F("Opublikowano Ciśn: ")); Serial.println(press,1);
  }
  if (jsonDoc.containsKey("WS")) {
    float ws = jsonDoc["WS"];
    mqttClient.publish(MQTT_TOPIC_WIND_SPEED, String(ws, 1).c_str(), true);
    Serial.print(F("Opublikowano Pręd.Wiatru: ")); Serial.println(ws,1);
  }
  if (jsonDoc.containsKey("WD")) {
    const char* wd = jsonDoc["WD"]; 
    mqttClient.publish(MQTT_TOPIC_WIND_DIRECTION, wd, true);
    Serial.print(F("Opublikowano Kier.Wiatru: ")); Serial.println(wd);
  }
  if (jsonDoc.containsKey("R")) {
    float rain = jsonDoc["R"];
    mqttClient.publish(MQTT_TOPIC_RAINFALL, String(rain, 2).c_str(), true);
    Serial.print(F("Opublikowano Opady: ")); Serial.println(rain,2);
  }
  updateDisplay(); 
  newLoRaPacketReceived = false; 
}

void setup() {
  Serial.begin(115200);
  while (!Serial); 
  Serial.println(F("LoRa to MQTT Receiver with OLED, HA Discovery & AS3935 (SparkFun) - Initializing..."));
  Serial.print(F("Wersja oprogramowania: ")); Serial.println(SW_VERSION);
  Wire.begin(OLED_SDA, OLED_SCL); 
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, "Inicjalizacja...");
  u8g2.sendBuffer();
  Serial.println(F("OLED zainicjalizowany."));
  delay(500);
  u8g2.enableUTF8Print(); 
  showDisplayMessage("Odbiornik Pogody", "Uruchamianie...", SW_VERSION, 1000); 
  setup_wifi();
  setup_as3935();
  if (mqttClient.setBufferSize(MQTT_PUBLISH_BUFFER_SIZE)) {
    Serial.print(F("Rozmiar bufora MQTT ustawiony na: ")); Serial.println(MQTT_PUBLISH_BUFFER_SIZE);
  } else {
    Serial.println(F("OSTRZEŻENIE: Nie udało się ustawić rozmiaru bufora MQTT. Może być za mały dla konfiguracji HA Discovery."));
    Serial.println(F("             Upewnij się, że biblioteka PubSubClient jest w wersji >= 2.8.0 lub zwiększ MQTT_MAX_PACKET_SIZE w PubSubClient.h"));
  }
  mqttClient.setServer(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
  displayMqttStatus = "MQTT: Connecting";
  updateDisplay();
  Serial.println(F("Inicjalizacja modułu LoRa..."));
  displayLoRaStatus = "LoRa: Init...";
  updateDisplay();
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println(F("BŁĄD: Uruchomienie LoRa nie powiodło się! Zatrzymywanie."));
    displayLoRaStatus = "LoRa: FAILED!";
    showDisplayMessage("LoRa Init Failed!", "Check Wiring", "Halting.", 0); 
    while (1);
  }

  
  LoRa.onReceive(onLoRaReceive_ISR);
  LoRa.receive(); 
  Serial.println(F("LoRa zainicjalizowany. Oczekiwanie na pakiety..."));
  displayLoRaStatus = "LoRa: Listening"; 
  updateDisplay();
  lastLoRaPacketMillis = millis();
  lastAs3935ActivityMillis = millis();
  AS3935_TUNING_CAP_VALUE = getBestTune();
}

void loop() {
  unsigned long currentTime = millis();
  if (!mqttClient.connected()) {
    reconnect_mqtt(); 
  }
  mqttClient.loop(); 
  if (newLoRaPacketReceived) {
    if (mqttClient.connected()) {
        processReceivedLoRaData(); 
    } else {
        Serial.println(F("Odebrano pakiet LoRa, ale MQTT nie jest połączony. Wstrzymuję przetwarzanie."));
    }
  }
  processAS3935Data(); 
  if (currentTime - lastDisplayUpdateTime >= displayUpdateInterval) {
      if (mqttClient.connected() && (displayMqttStatus != "MQTT: Connected" && !displayMqttStatus.startsWith("MQTT: Fail"))) {
          displayMqttStatus = "MQTT: Connected";
      } else if (!mqttClient.connected() && displayMqttStatus != "MQTT: FAILED" && displayMqttStatus != "MQTT: Connecting" && !displayMqttStatus.startsWith("MQTT: Fail")) {
          displayMqttStatus = "MQTT: Disconnected";
      }
      if (displayLoRaStatus.startsWith("LoRa: RSSI") && (currentTime - lastLoRaPacketMillis > 5000)) { 
          displayLoRaStatus = "LoRa: Listening";
      } else if (displayLoRaStatus != "LoRa: JSON Err" && displayLoRaStatus != "LoRa: FAILED!" &&
                 displayLoRaStatus != "LoRa: Init..." && !displayLoRaStatus.startsWith("LoRa: RSSI") &&
                 !newLoRaPacketReceived) { 
          displayLoRaStatus = "LoRa: Listening";
      }
      if (!displayAs3935Status.endsWith("Listening") && !displayAs3935Status.endsWith("Init...") && !displayAs3935Status.endsWith("FAILED!")) {
        if (currentTime - lastAs3935ActivityMillis > 30000) { 
            displayAs3935Status = "AS3935: Listening";
        }
      }
      updateDisplay();
      lastDisplayUpdateTime = currentTime;
  }
  delay(10); 
}
