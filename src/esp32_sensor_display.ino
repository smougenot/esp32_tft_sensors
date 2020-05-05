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

#include <Adafruit_NeoPixel.h>

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

// Neopixel settings ------------------------------------------------------

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN 17

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 12

// Alignment of the pixel orientation (depends on the wireing + mounting)
#define PIXEL_SHIFT 0

// Hour animation settings

// Number of times the hour display repeats
#define HOUR_REPEAT 2
// Delay between frames during background roll
#define BACKGROUND_FRAME_WAIT 100
// Delay between frames during foreground roll (hour counting)
#define FOREGROUND_FRAME_WAIT 600
// Delay at the end of the animation
#define END_WAIT 1000

// Neopixel Variables -----------------------------------------------------

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

// Colors schemes ----------------------------------------------------------

// Basics

const uint32_t color_off = strip.Color(0, 0, 0);
const uint32_t color_white = strip.Color(85, 85, 85);
const uint32_t color_blue = strip.Color(0, 0, 255);
const uint32_t color_green = strip.Color(0, 255, 0);
const uint32_t color_red = strip.Color(255, 0, 0);

//tequila sunrise color scheme

const uint32_t color_orange = strip.Color(173, 82, 0);              // redest orange
const uint32_t color_slightly_yellower = strip.Color(152, 103, 0);  //slightly yellower
const uint32_t color_yellow = strip.Color(128, 128, 0);             //yellow
const uint32_t color_redish = strip.Color(228, 0, 27);              //red

//blue, green, & purple color scheme

const uint32_t color_magenta = strip.Color(128, 0, 128);    // magenta
const uint32_t color_purple = strip.Color(71, 0, 184);      // purple
const uint32_t color_royal_blue = strip.Color(0, 47, 208);  // royal blue
const uint32_t color_greenish = strip.Color(0, 208, 47);    // green

// Colors used -------------------------------------------------------------

// One Instance of color settings for the clock
class ColorScheme {
 public:
  uint32_t background;
  uint32_t foreground;

  ColorScheme(uint32_t aBackground, uint32_t aForeground);
  void display();
};

ColorScheme::ColorScheme(uint32_t aBackground, uint32_t aForeground) {
  background = aBackground;
  foreground = aForeground;
}

// Do not use when Serial is not ready
void ColorScheme::display() {
  Serial.print("ColorScheme ");
  Serial.print(background);
  Serial.print(" ");
  Serial.println(foreground);
}

ColorScheme color_scheme_bedtime = ColorScheme(
    color_magenta,
    color_royal_blue);
ColorScheme color_scheme_sleep = ColorScheme(
    color_royal_blue,
    color_magenta);
ColorScheme color_scheme_wake = ColorScheme(
    color_royal_blue,
    color_greenish);
ColorScheme color_scheme_day = ColorScheme(
    color_orange,
    color_yellow);

// Colors to use according to the hour of the day
// TODO: change for a more clever way (more accurate changes, less redondancy)
ColorScheme color_schemes[24] = {
    color_scheme_sleep,    // 00h
    color_scheme_sleep,    // 01h
    color_scheme_sleep,    // 02h
    color_scheme_sleep,    // 03h
    color_scheme_sleep,    // 04h
    color_scheme_sleep,    // 05h
    color_scheme_sleep,    // 06h
    color_scheme_wake,     // 07h
    color_scheme_wake,     // 08h
    color_scheme_wake,     // 09h
    color_scheme_wake,     // 10h
    color_scheme_day,      // 11h
    color_scheme_day,      // 12h
    color_scheme_day,      // 13h
    color_scheme_day,      // 14h
    color_scheme_day,      // 15h
    color_scheme_day,      // 16h
    color_scheme_day,      // 17h
    color_scheme_day,      // 18h
    color_scheme_day,      // 19h
    color_scheme_bedtime,  // 20h
    color_scheme_bedtime,  // 21h
    color_scheme_sleep,    // 22h
    color_scheme_sleep,    // 23h
};

// Variables --------------------------------------------------------------

WMSettings settings;

// I2C pins defaults (change if required)
// #define IC_CLK  (5)
// #define IC_DATA (4)
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

// time
const char* ntpServer = "fr.pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
struct tm timeinfo;

// Writes time on Serial
void printLocalTime() {
  if(!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

//! Long time delay, it is recommended to use shallow sleep, which can
//! effectively reduce the current consumption
void espDelay(int ms) {
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}

// Settings management ----------------------------------------------------

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
  WiFiManager wm;
  wm.resetSettings();

  WMSettings blank;
  settings = blank;
  configSave();
}


// Init -------------------------------------------------------------------

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
  Serial.println("I2C scan started");
  for (byte addr = from_addr; addr <= to_addr; addr++) {
    Wire.beginTransmission(addr);
    rc = Wire.endTransmission();
    callback(addr, rc);
  }
  Serial.println("\nI2C scan ended");
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
//  mqttReconnect();
}

void mqTTCheckClient() {
  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();
}

void mqttReconnect() {
  Serial.println("reconnect");

  //reconnect wifi first
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("  no Wifi,starting it");
    initWifiManager();
    if (WiFi.status() != WL_CONNECTED) {
      ESP.restart();
    }
  }

  int cpt = 2;

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

void initStrip() {
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(5);  // Set BRIGHTNESS (max = 255)
}

// Button management ------------------------------------------------------

void button_init() {
  btn1.setPressedHandler([](Button2 &b) {
    Serial.println("Detect Voltage..");
    btn1Clicked = !btn1Clicked;
  });

  btn2.setPressedHandler([](Button2 &b) {
    Serial.println("btn press both for config reset");
    if(btn2.isPressed() && btn1.isPressed()) {
      resetSettings();
      ESP.restart();
    }
  });
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

  // Confire local time references
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  // Neopixel strip
  initStrip();
}

// Main loop --------------------------------------------------------------

void button_loop() {
  btn1.loop();
  btn2.loop();
}

// Reads voltage and dispays it on TFT
// Updates every seconds
void showVoltage() {
  static uint64_t timeStamp = 0;
  if (millis() - timeStamp > 1000) {
    timeStamp = millis();
    readVoltage();
    displayVoltage(battery_voltage);
  }
}

void loop() {
  button_loop();
  long now = millis();
  if (now - lastCheck > LOOP_WAIT) {
    lastCheck = now;
    printLocalTime();
    displayReset();
    displayTime();

    if (btn1Clicked) {
      showVoltage();
    } else {
      if (!bme_available) {
        scanI2CBus(i2c_scan_start_address, i2c_scan_end_address, scanFunc);
      } else {
        readSensor();
        readVoltage();
        displaySensor(temperature, humidity, battery_voltage);
        publishStatus();
      }
    }

    // Animate hour on neopixel strip
    displayHour(timeinfo.tm_hour);

    // Deep sleep to ave energy
    // espDelay(LOOP_WAIT);
  }
}

// Data push --------------------------------------------------------------

void publishStatus() {
  mqTTCheckClient();
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

// Sensors read -----------------------------------------------------------

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

// TFT display ------------------------------------------------------------

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

void displayReset() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setRotation(0);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
}

void displaySensor(float aTemperature, float aHumidity, float aVoltage) {
  tft.setTextSize(3);
  tft.setCursor(0, 0);

  sprintf(buff, "%.2f C\n%.2f %%\n %.2f V", aTemperature, aHumidity, aVoltage);
  tft.println(buff);
}

void displayVoltage(float aVoltage) {
    String voltage = String(aVoltage) + "V";
    Serial.println(voltage);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(voltage, 0, 0);
}

void displayTime() {
  Serial.println("Display time");

  tft.setTextColor(TFT_OLIVE, TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(25, 150);

  sprintf(buff, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  tft.println(buff);
}

// Some functions of to display time on neopixel strip --------------------

// Animation,
// Display hour
// Choose colors according to settings
// repeat according to settings
// - hourOfTheDay, the hour number (0-24)
void displayHour(uint16_t hourOfTheDay) {
  Serial.print("Begin displayHour ");
  Serial.println(hourOfTheDay);

  ColorScheme colors = color_schemes[hourOfTheDay % 24];
  for (int i = 0; i < HOUR_REPEAT; i++) {
    displayHour(colors.background, colors.foreground, hourOfTheDay);
    rainbow(1, 5);
  }
}

// Animation,
// First, wipes the clock using background color
// Second, one by one, turns on the amout of hours
void displayHour(uint32_t backgroundcolor, uint32_t foregroundcolor, uint16_t hour) {
  Serial.print("Begin displayHour ");
  Serial.print(backgroundcolor);
  Serial.print(" ");
  Serial.print(foregroundcolor);
  Serial.print(" ");
  Serial.println(hour);

  strip.fill(color_white, 0, 0);
  colorWipe(backgroundcolor, BACKGROUND_FRAME_WAIT);                // Fill the whole clock
  delay(FOREGROUND_FRAME_WAIT);                                     // pause
  colorWipe(foregroundcolor, hour % 12, 1, FOREGROUND_FRAME_WAIT);  // Fill the hours only
  delay(END_WAIT);                                                  // pause
}

// Some functions of our own for creating animated effects ----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  colorWipe(color, strip.numPixels(), 0, wait);
}

// Covered n first pixel of the strip one after another with a color.
// Strip is NOT cleared
// first; anything there will be covered pixel by pixel.
// - color,
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// - length, number of pixels
// - offset, position of the first pixel to change
// - Wait, delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, uint16_t length, uint16_t offset, int wait) {
  Serial.print("Begin colorWipe ");
  Serial.print(color);
  Serial.print(" ");
  Serial.print(length);
  Serial.print(" ");
  Serial.print(offset);
  Serial.print(" ");
  Serial.println(wait);

  for (int i = 0; i < min(length, strip.numPixels()); i++) {  // For each pixel in strip...
    strip.setPixelColor(
        (i + offset + PIXEL_SHIFT) % strip.numPixels(),
        color);    //  Set pixel's color (in RAM)
    strip.show();  //  Update strip to match
    delay(wait);   //  Pause for a moment
  }
}

// Rainbow cycle along whole strip.
// Full Rotation is 256 frames.
// Lap, number of full rotations
// wait, delay time (in ms) between frames.
void rainbow(int lap, int wait) {
  Serial.print("Begin rainbow ");
  Serial.print(lap);
  Serial.print(" ");
  Serial.println(wait);

  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for (long firstPixelHue = 0; firstPixelHue < lap * 65536; firstPixelHue += 256) {
    for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show();  // Update strip with new contents
    delay(wait);   // Pause for a moment
  }
}

