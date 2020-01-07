#include <TFT_eSPI.h>
#include <SPI.h>
#include "WiFi.h"
#include <Wire.h>
#include <Button2.h>
#include "esp_adc_cal.h"
#include "bmp.h"
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager

// I2C sensors
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

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

// I2C pins defaults on TTGO-Display
// #define IC_CLK  22
// #define IC_DATA 21
// In case no device is responding, initiate a I2C bus scan
byte i2c_scan_start_address = 8; // lower addresses are reserved to prevent conflicts with other protocols
byte i2c_scan_end_address = 119; // higher addresses unlock other modes, like 10-bit addressing

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

Adafruit_BME280 bme;
boolean bme_available = false;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

char buff[512];
int vref = 1100;
int btnClick = false;

// Datas
float temperature = -1;
float pressure = -1;
float humidity = -1;
float battery_voltage = -1;

short loopCnt = 0;
// timing
long lastCheck = 0;

char chipId[23];

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}

void showVoltage()
{
  static uint64_t timeStamp = 0;
  if (millis() - timeStamp > 1000)
  {
    timeStamp = millis();
    readVoltage();
    String voltage = "Voltage :" + String(battery_voltage) + "V";
    Serial.println(voltage);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(voltage, tft.width() / 2, tft.height() / 2);
  }
}

void button_init()
{
  btn1.setLongClickHandler([](Button2 &b) {
    btnClick = false;
    int r = digitalRead(TFT_BL);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Press again to wake up", tft.width() / 2, tft.height() / 2);
    espDelay(6000);
    digitalWrite(TFT_BL, !r);

    tft.writecommand(TFT_DISPOFF);
    tft.writecommand(TFT_SLPIN);
    esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_deep_sleep_start();
  });
  btn1.setPressedHandler([](Button2 &b) {
    Serial.println("Detect Voltage..");
    btnClick = true;
  });

  btn2.setPressedHandler([](Button2 &b) {
    btnClick = false;
    Serial.println("btn press wifi scan");
    wifi_scan();
  });
}

void button_loop()
{
  btn1.loop();
  btn2.loop();
}

void wifi_scan()
{
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setRotation(1);
  tft.setTextSize(2);

  tft.drawString("Scan Network", tft.width() / 2, tft.height() / 2);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  int16_t n = WiFi.scanNetworks();
  tft.fillScreen(TFT_BLACK);
  if (n == 0)
  {
    tft.drawString("no networks found", tft.width() / 2, tft.height() / 2);
  }
  else
  {
    tft.setTextDatum(TL_DATUM);
    tft.setCursor(0, 0);
    Serial.printf("Found %d net\n", n);
    for (int i = 0; i < n; ++i)
    {
      sprintf(buff,
              "[%d]:%s(%d)",
              i + 1,
              WiFi.SSID(i).c_str(),
              WiFi.RSSI(i));
      tft.println(buff);
    }
  }
  WiFi.mode(WIFI_OFF);
}

void initChipId()
{
  uint64_t chipMac = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  snprintf(chipId, 23, "%04X%08X", (uint16_t)(chipMac >> 32), (uint32_t)chipMac);
}

void initTft()
{
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
  if (TFT_BL > 0)
  {                                         // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    pinMode(TFT_BL, OUTPUT);                // Set backlight pin to output mode
    digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
  }

  Serial.println("tft swap bytes");
  tft.setSwapBytes(true);
  Serial.println("tft push image");
  tft.pushImage(0, 0, 240, 135, ttgo);
  espDelay(250);

  Serial.println("tft colors");
  tft.setRotation(0);
  int i = 5;
  while (i--)
  {
    tft.fillScreen(TFT_RED);
    espDelay(250);
    tft.fillScreen(TFT_BLUE);
    espDelay(250);
    tft.fillScreen(TFT_GREEN);
    espDelay(250);
  }
  tft.fillScreen(TFT_BLACK);
}

void initVref()
{
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
  {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  }
  else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
  {
    Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  }
  else
  {
    Serial.println("Default Vref: 1100mV");
  }
}

// Scan the I2C bus
// On each address, call the callback function with the address and result.
// If result==0, address has a device
void scanI2CBus(byte from_addr, byte to_addr,
                void (*callback)(byte address, byte result))
{
  byte rc;
  for (byte addr = from_addr; addr <= to_addr; addr++)
  {
    Wire.beginTransmission(addr);
    rc = Wire.endTransmission();
    callback(addr, rc);
  }
}

// Called when address is scanned scanI2CBus()
// If result==0, address has a device
void scanFunc(byte addr, byte result)
{
  Serial.print("addr: ");
  Serial.print(addr, HEX);
  Serial.print((result == 0) ? " found!" : "       ");
  Serial.print((addr % 4) ? "\t" : "\n");
}

void initBME()
{
  if (!bme.begin(0x76, &Wire))
  {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"));
    Serial.print(F("SensorID was: 0x"));
    Serial.println(bme.sensorID(), 16);
    Serial.print(F("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n"));
    Serial.print(F("   ID of 0x56-0x58 represents a BMP 280,\n"));
    Serial.print(F("        ID of 0x60 represents a BME 280.\n"));
    Serial.print(F("        ID of 0x61 represents a BME 680.\n"));
    Serial.println(F("Will process to I2C scan"));
    bme_available = false;
  }
  else
  {
    bme_available = true;
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
}

void setup()
{
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
}

void loop()
{
  button_loop();
  long now = millis();
  if (now - lastCheck > LOOP_WAIT)
  {
    lastCheck = now;
    if (btnClick)
    {
      showVoltage();
    }
    if (!bme_available)
    {
      scanI2CBus(i2c_scan_start_address, i2c_scan_end_address, scanFunc);
    }
    else
    {
      readSensor();
      readVoltage();
      displaySensor(temperature, humidity, battery_voltage);
      delay(1000);
    }
    espDelay(LOOP_WAIT);
  }
}

void readVoltage()
{
  uint16_t v = analogRead(ADC_PIN);
  battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
}

void readSensor()
{
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

void displaySensor(float aTemperature, float aHumidity, float aVoltage)
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setRotation(0);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);

  sprintf(buff,
          " %.2f C\n %.2f %%\n  %.2f V",
          aTemperature,
          aHumidity,
          aVoltage);
  tft.println(buff);
}
