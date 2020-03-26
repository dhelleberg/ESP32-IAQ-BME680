#include <Arduino.h>
#include "WiFi.h"
#include <U8g2lib.h>
#include <Wire.h>
#include "bsec.h"
#include <EEPROM.h>
#include <MQTT.h>
#include <updater.h>

/*Put your SSID & Password*/
const char *ssid = WSSID;    // Enter SSID here
const char *password = WPWD; //Enter Password here

const int WIFI_TIMEOUT_RESTART_S = 60;

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);

const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_28d/bsec_iaq.txt"
};
#define STATE_SAVE_PERIOD UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

// Create an object of the class Bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

String output;

int displayUpdateCount = 0;
int counter = 0;

long lastUpdateCheck =  0;
long lastReconnect =  0;

WiFiClient net;
MQTTClient client;

String uint64ToString(uint64_t input)
{
  String result = "";
  uint8_t base = 10;

  do
  {
    char c = input % base;
    input /= base;

    if (c < 10)
      c += '0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 15, 4, 16);
void setup()
{
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length
  Serial.begin(115200);

  // Set WiFi to station mode and disconnect from an AP if it was previously connected

  // Start Wifi connection
  WiFi.mode(WIFI_OFF);
  WiFi.setSleep(false);

  WiFi.begin(ssid, password);
  Serial.print("[WIFI] Connecting to WiFi ");

  int wifi_connection_time = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
    wifi_connection_time++;
    if (wifi_connection_time > WIFI_TIMEOUT_RESTART_S)
    {
      Serial.println("[WIFI] Run into Wifi Timeout, try restart");
      Serial.flush();
      WiFi.disconnect();
      ESP.restart();
    }
  }
  Serial.println("\n[WIFI] Connected");
  lastReconnect = millis();
  Updater::check_for_update();
  Wire.begin();
  u8g2.begin();

  Wire.begin(21, 22);

  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  iaqSensor.setTemperatureOffset(0.7);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();
  loadState();
  bsec_virtual_sensor_t sensorList[10] = {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);
}

void displayText(String text1, String text2, String text3)
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g_font_helvB10);
  u8g2.drawStr(0, 20, text1.c_str());
  u8g2.drawStr(0, 40, text2.c_str());
  u8g2.drawStr(0, 60, text3.c_str());

  u8g2.sendBuffer();
}

void loop()
{
  unsigned long time_trigger = millis();
  if (iaqSensor.run())
  { // If new data is available
    output = String(time_trigger);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    output += ",i: " + String(iaqSensor.iaq);
    output += ",iA: " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    output += ",sI " + String(iaqSensor.staticIaq);
    output += ",co2: " + String(iaqSensor.co2Equivalent);
    output += ",voc: " + String(iaqSensor.breathVocEquivalent);
    Serial.println(output);
    if (counter % 4 == 0)
      displayText(String(iaqSensor.temperature) + " C", String(iaqSensor.iaq) + " Q: " + String(iaqSensor.iaqAccuracy), String(iaqSensor.co2Equivalent) + " co2 " + String(iaqSensor.breathVocEquivalent) + " voc");
    counter++;
    updateState();
    Serial.println("next: " + uint64ToString(iaqSensor.nextCall));
  }
  else
  {
    checkIaqSensorStatus();
  }
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK)
  {
    if (iaqSensor.status < BSEC_OK)
    {
      output = "Sensor BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    }
    else
    {
      output = "Sensor BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK)
  {
    if (iaqSensor.bme680Status < BME680_OK)
    {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    }
    else
    {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void errLeds(void)
{
  Serial.println("Error");
  delay(5000);
}

void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
  {
    // Existing state in EEPROM
    Serial.println(">>>>>>>>>Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  }
  else
  {
    // Erase the EEPROM with zeroes
    Serial.println(">>>>>>>>>Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}
void updateState(void)
{
  bool update = false;
  /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
  if (stateUpdateCounter == 0)
  {
    if (iaqSensor.iaqAccuracy >= 3)
    {
      update = true;
      stateUpdateCounter++;
    }
  }
  else
  {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis())
    {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update)
  {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println(">>>>>>>>>Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}