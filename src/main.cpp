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
const char *mqttprefix= MQTT_ID;

const int UPDATE_INTERVAL = 60 * 1000 * 10; //every 10 min
const int RECONNECT_INTERVAL = 30 * 1000;

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
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 15, 4, 16);

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
void displayText(String text1, String text2, String text3)
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g_font_helvB10);
  u8g2.drawStr(0, 20, text1.c_str());
  u8g2.drawStr(0, 40, text2.c_str());
  u8g2.drawStr(0, 60, text3.c_str());

  u8g2.sendBuffer();
}


void setup()
{
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length
  Serial.begin(115200);
  Wire.begin();
  u8g2.begin();

  Wire.begin(21, 22);

  displayText("init...","","");
  // Set WiFi to station mode and disconnect from an AP if it was previously connected

  // Start Wifi connection
  WiFi.mode(WIFI_OFF);
  WiFi.setSleep(false);

  WiFi.begin(ssid, password);
  Serial.print("[WIFI] Connecting to WiFi ");
  displayText("init...","starting wifi","");
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
  displayText("init...","connected","wait for sensor...");
  Serial.println("\n[WIFI] Connected");
  lastReconnect = millis();
  Updater::check_for_update();

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

  Serial.print("\n mqtt.. connecting...");
  client.begin("192.168.178.21",net);
  
  while (!client.connect("esp-iaq", "try", "try")) {
    Serial.print(".");
    delay(1000);
  }
  client.publish(String(mqttprefix)+"fw-version", FIRMWARE_VERSION, true, 2);


  Serial.println("\nconnected!");
}


void loop()
{
  client.loop(); //loop mqtt client
  //handle re-connects 
  long now = millis();
  if(now > (lastReconnect + RECONNECT_INTERVAL)) {
    lastReconnect = now;
    if(WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WIFI re-connecting...");
      int wifi_retry = 0;
      while(WiFi.status() != WL_CONNECTED && wifi_retry < 5 ) {
        wifi_retry++;
        Serial.println("WiFi not connected. Try to reconnect");
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        delay(100);
      }
    }
    if (!client.connected()) {
        Serial.println("mqtt client re-connecting...");
        int tries = 0;
        boolean repeat = true;
        while (!client.connect("esp-iaq", "try", "try") && repeat) {
          Serial.print(".");
          delay(100);
          tries++;
          if(tries > 5)
            repeat = false;
        }
    }
  }
  if(now > (lastUpdateCheck+UPDATE_INTERVAL)) {

    Serial.println("checking for SW-Update");
    Updater::check_for_update();
    lastUpdateCheck = now;
  }


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

    client.publish(String(mqttprefix)+"pressure", String(iaqSensor.pressure), true, 2);
    client.publish(String(mqttprefix)+"gasResistance", String(iaqSensor.gasResistance), true, 2);
    client.publish(String(mqttprefix)+"iaq", String(iaqSensor.iaq), true, 2);
    client.publish(String(mqttprefix)+"iaqAccuracy", String(iaqSensor.iaqAccuracy), true, 2);
    client.publish(String(mqttprefix)+"temperature", String(iaqSensor.temperature), true, 2);
    client.publish(String(mqttprefix)+"humidity", String(iaqSensor.humidity), true, 2);
    client.publish(String(mqttprefix)+"staticIaq", String(iaqSensor.staticIaq), true, 2);
    client.publish(String(mqttprefix)+"staticIaqAccuracy", String(iaqSensor.staticIaqAccuracy), true, 2);
    client.publish(String(mqttprefix)+"co2Equivalent", String(iaqSensor.co2Equivalent), true, 2);
    client.publish(String(mqttprefix)+"breathVocEquivalent", String(iaqSensor.breathVocEquivalent), true, 2);
    client.publish(String(mqttprefix)+"breathVocAccuracy", String(iaqSensor.breathVocAccuracy), true, 2);
    client.publish(String(mqttprefix)+"co2Accuracy", String(iaqSensor.co2Accuracy), true, 2);
    client.publish(String(mqttprefix)+"compGasAccuracy", String(iaqSensor.compGasAccuracy), true, 2);
    client.publish(String(mqttprefix)+"compGasValue", String(iaqSensor.compGasValue), true, 2);
    client.publish(String(mqttprefix)+"gasPercentage", String(iaqSensor.gasPercentage), true, 2);
    client.publish(String(mqttprefix)+"gasPercentageAcccuracy", String(iaqSensor.gasPercentageAcccuracy), true, 2);
    
    
    if (counter % 4 == 0)
      displayText(String(iaqSensor.temperature) + " C", String(iaqSensor.iaq) + " Q: " + String(iaqSensor.iaqAccuracy)+ " "+String(iaqSensor.staticIaq), String(iaqSensor.co2Equivalent) + " co2 ");
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