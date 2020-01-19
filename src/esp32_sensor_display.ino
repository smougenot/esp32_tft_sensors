#include <Button2.h>
#include <EEPROM.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFiManager.h>
#include <Wire.h>
#include "WiFi.h"
#include "bmp.h"
#include "esp_adc_cal.h"

// I2C sensors
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// send MQTT messages
#include <PubSubClient.h>

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN 0x10
#endif

#define ADC_EN 14
#define ADC_PIN 34
#define BUTTON_1 35
#define BUTTON_2 0

// wait time (ms)
#define LOOP_WAIT 6 * 1000

// MQTT Settings
#define EEPROM_SALT 311276  // to check version
typedef struct {
  char mqtt_server[120] = "";
  char mqtt_port[6] = "1883";
  char mqtt_user[120] = "";
  char mqtt_password[120] = "";
  int salt = EEPROM_SALT;
} WMSettings;

WMSettings settings;

// I2C pins defaults on TTGO-Display
// #define IC_CLK  22
// #define IC_DATA 21
// In case no device is responding, initiate a I2C bus scan
byte i2c_scan_start_address = 8;  // lower addresses are reserved to prevent conflicts with other protocols
byte i2c_scan_end_address = 119;  // higher addresses unlock other modes, like 10-bit addressing

TFT_eSPI tft = TFT_eSPI(135, 240);  // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

Adafruit_BME280 bme;
boolean bme_available = false;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

char buff[512];
int vref = 1100;
int btn1Clicked = false;

// Datas
float temperature = -1;
float pressure = -1;
float humidity = -1;
float battery_voltage = -1;

short loopCnt = 0;
// timing
long lastCheck = 0;

char chipId[23];

String topicCmd;
String topicStatus;
String mqttClientId;

WiFiClient espClient;
PubSubClient client(espClient);

//! Long time delay, it is recommended to use shallow sleep, which can
//! effectively reduce the current consumption
void espDelay(int ms) {
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}

void showVoltage() {
  static uint64_t timeStamp = 0;
  if (millis() - timeStamp > 1000) {
    timeStamp = millis();
    readVoltage();
    String voltage = "Voltage :" + String(battery_voltage) + "V";
    Serial.println(voltage);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(voltage, tft.width() / 2, tft.height() / 2);
  }
}

void button_init() {
  btn1.setPressedHandler([](Button2 &b) {
    Serial.println("Detect Voltage..");
    btn1Clicked != btn1Clicked;
  });

  btn2.setPressedHandler([](Button2 &b) {
    Serial.println("btn press config reset");
    // resetSettings();
    WiFiManager wm;
    wm.resetSettings();
    ESP.restart();
  });
}

void button_loop() {
  btn1.loop();
  btn2.loop();
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  displayWifiManager(myWiFiManager->getConfigPortalSSID(), WiFi.softAPIP());
}

void saveWifiCallback() {
  Serial.println("[CALLBACK] save Wifi callback fired");
}

void saveParamCallback() { Serial.println("[CALLBACK] param callback fired"); }

void configLoad() {
  Serial.println("Loading config");

  EEPROM.begin(512);
  EEPROM.get(0, settings);
  EEPROM.end();

  Serial.println("Config loaded");
}

void configSave() {
  Serial.println("Saving config");

  EEPROM.begin(512);
  EEPROM.put(0, settings);
  if (EEPROM.commit()) {
    Serial.println("Config saved");
  } else {
    Serial.println("EEPROM error");
  }
  EEPROM.end();
}

void resetSettings() {
  WMSettings blank;
  settings = blank;
  configSave();
}

void initWifiManager() {
  Serial.println("initWifiManager");
  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveWifiCallback);
  wifiManager.setSaveParamsCallback(saveParamCallback);

  // Custom params for MQTT
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", settings.mqtt_server, 120);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", settings.mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", settings.mqtt_user, 120);
  WiFiManagerParameter custom_mqtt_password("password", "mqtt password", settings.mqtt_password, 120);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_password);

  // set static ip
  // wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0)); // set static ip,gw,sn
  // wifiManager.setShowStaticFields(true); // force show static ip fields
  // wifiManager.setShowDnsFields(true);    // force show dns field always

  wifiManager.setConfigPortalTimeout(120);  // auto close configportal after n seconds

  wifiManager.setCaptivePortalEnable(false);  // disable captive portal redirection
  // wifiManager.setAPClientCheck(true); // avoid timeout if client connected to softap

  // wifi scan settings
  // wifiManager.setRemoveDuplicateAPs(false); // do not remove duplicate ap names (true)
  // wifiManager.setMinimumSignalQuality(20);  // set min RSSI (percentage) to show in scans, null = 8%
  // wifiManager.setShowInfoErase(false);      // do not show erase button on info page
  // wifiManager.setScanDispPerc(true);       // show RSSI as percentage not graph icons

  // set dark theme
  wifiManager.setClass("invert");

  String ssid = "ESP_autoconnect_" + String(chipId);
  String pwd = "password";

  if (!wifiManager.autoConnect(ssid.c_str(), pwd.c_str())) {
    Serial.println("failed to connect and hit timeout");
    espDelay(3000);
    // reset and try again, or maybe put it to deep sleep
    ESP.restart();
  }

  Serial.println("Reading config");

  strcpy(settings.mqtt_server, custom_mqtt_server.getValue());
  strcpy(settings.mqtt_port, custom_mqtt_port.getValue());
  strcpy(settings.mqtt_user, custom_mqtt_user.getValue());
  strcpy(settings.mqtt_password, custom_mqtt_password.getValue());

  configSave();
}

void initChipId() {
  uint64_t chipMac = ESP.getEfuseMac();  // The chip ID is essentially its MAC address(length: 6 bytes).
  snprintf(chipId, 23, "%04X%08X", (uint16_t)(chipMac >> 32),
           (uint32_t)chipMac);
}

void initTft() {
  tft.init();
  tft.setRotation(1);
  Serial.println("tft to black");
  tft.fillScreen(TFT_BLACK);
  Serial.println("tft text");
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);

  Serial.println("tft backlight");
  if (TFT_BL > 0) {           // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    pinMode(TFT_BL, OUTPUT);  // Set backlight pin to output mode
    digitalWrite(TFT_BL,
                 TFT_BACKLIGHT_ON);  // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
  }

  Serial.println("tft swap bytes");
  tft.setSwapBytes(true);
  Serial.println("tft push image");
  tft.pushImage(0, 0, 240, 135, ttgo);
  espDelay(250);

  Serial.println("tft colors");
  tft.setRotation(0);
  int i = 5;
  while (i--) {
    tft.fillScreen(TFT_RED);
    espDelay(250);
    tft.fillScreen(TFT_BLUE);
    espDelay(250);
    tft.fillScreen(TFT_GREEN);
    espDelay(250);
  }
  tft.fillScreen(TFT_BLACK);
}

void initVref() {
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
      (adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6,
      (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  // Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n",
                  adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
    Serial.println("Default Vref: 1100mV");
  }
}

// Scan the I2C bus
// On each address, call the callback function with the address and result.
// If result==0, address has a device
void scanI2CBus(byte from_addr, byte to_addr,
                void (*callback)(byte address, byte result)) {
  byte rc;
  for (byte addr = from_addr; addr <= to_addr; addr++) {
    Wire.beginTransmission(addr);
    rc = Wire.endTransmission();
    callback(addr, rc);
  }
}

// Called when address is scanned scanI2CBus()
// If result==0, address has a device
void scanFunc(byte addr, byte result) {
  Serial.print("addr: ");
  Serial.print(addr, HEX);
  Serial.print((result == 0) ? " found!" : "       ");
  Serial.print((addr % 4) ? "\t" : "\n");
}

void initBME() {
  if (!bme.begin(0x76, &Wire)) {
    Serial.println(
        F("Could not find a valid BME280 sensor, check wiring, address, sensor "
          "ID!"));
    Serial.print(F("SensorID was: 0x"));
    Serial.println(bme.sensorID(), 16);
    Serial.print(
        F("        ID of 0xFF probably means a bad address, a BMP 180 or BMP "
          "085\n"));
    Serial.print(F("   ID of 0x56-0x58 represents a BMP 280,\n"));
    Serial.print(F("        ID of 0x60 represents a BME 280.\n"));
    Serial.print(F("        ID of 0x61 represents a BME 680.\n"));
    Serial.println(F("Will process to I2C scan"));
    bme_available = false;
  } else {
    bme_available = true;
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
}

void initMqtt() {
  mqttClientId = "ESP32Client" + String(chipId);
  topicCmd = "esp/" + String(chipId) + "/cmd";
  topicStatus = "esp/" + String(chipId) + "/status";

  client.setServer(settings.mqtt_server, String(settings.mqtt_port).toInt());
  mqttReconnect();
}

void mqttReconnect() {
  Serial.println("reconnect");

  //reconnect wifi first
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("  no Wifi : reset");
    return;
  }

  int cpt = 5;

  // Loop until we're reconnected to MQTT server
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttClientId.c_str(), settings.mqtt_user, settings.mqtt_password)) {
      Serial.print("connected with id : ");
      Serial.println(mqttClientId);
      // Once connected, publish an announcement...
      client.publish(topicStatus.c_str(), "hello world");
      // Serial.print("subscribing to : ");
      // Serial.println(topicCmd);
      // if (client.subscribe(topicCmd.c_str())) {
      //   Serial.println(" succeeded");
      // } else {
      //   Serial.println(" failed");
      // }
    } else {
      if (--cpt == 0) {
        Serial.println("  no MQTT : reset");
        return;
      }

      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      espDelay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start");

  initChipId();
  Serial.print("Chip ID : ");
  Serial.println(chipId);

  Serial.println("tft init");
  initTft();

  Serial.println("button init");
  button_init();

  Serial.println("vref init");
  initVref();

  Serial.println("Sensor init");
  // Wire.begin(IC_DATA, IC_CLK);
  initBME();

  configLoad();

  Serial.println("WifiManager init");
  initWifiManager();

  // Init MQTT client
  initMqtt();
}

void loop() {
  button_loop();
  long now = millis();
  if (now - lastCheck > LOOP_WAIT) {
    lastCheck = now;
    if (btn1Clicked) {
      showVoltage();
    } else {
      if (!bme_available) {
        scanI2CBus(i2c_scan_start_address, i2c_scan_end_address, scanFunc);
      } else {
        readSensor();
        readVoltage();
        displaySensor(temperature, humidity, battery_voltage);
        mqttReconnect();
        publishStatus();
      }
    }
    // Deep sleep to ave energy
    espDelay(LOOP_WAIT);
  }
}

void publishStatus() {
  if(client.connected()){
    Serial.println("publishing data to MQTT");
    publish("battery", battery_voltage);
    publish("temperature", temperature);
    publish("humidity", humidity);
    publish("pressure", pressure);
  } else {
    Serial.println("No MQTT channel available to publish data");
  }
}

void publish(const char* statusSuffix, float value) {
    char topic[50];
    char payload[50];
    sprintf(topic, "%s/%s", topicStatus.c_str(), statusSuffix);
    sprintf(payload, "%.2f", value);

    // Serial.print("topic: ");
    // Serial.print(topic);
    // Serial.print(" payload: ");
    // Serial.println(payload);

    client.publish(topic, payload);
}

void readVoltage() {
  uint16_t v = analogRead(ADC_PIN);
  battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
}

void readSensor() {
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  temperature = temp_event.temperature;
  humidity = humidity_event.relative_humidity;
  pressure = pressure_event.pressure;

  Serial.print(F("Temperature = "));
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print(F("Humidity = "));
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print(F("Pressure = "));
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.println();
}

void displayWifiManager(String wifiAP, IPAddress configAddress) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setRotation(1);
  tft.setTextColor(TFT_DARKCYAN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);

  sprintf(buff, "%s\n%s", wifiAP.c_str(), configAddress.toString().c_str());
  tft.println(buff);
}

void displaySensor(float aTemperature, float aHumidity, float aVoltage) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setRotation(0);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(0, 0);

  sprintf(buff, "%.2f C\n%.2f %%\n %.2f V", aTemperature, aHumidity, aVoltage);
  tft.println(buff);
}
