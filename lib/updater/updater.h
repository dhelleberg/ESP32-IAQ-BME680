#if !defined(UPDATER_H)
#define UPDATER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>

#define UPDATER_VERSION_URL \
  "http://192.168.178.21:81/ota-iaq.json"
#define UPDATER_FIRMWARE_URL \
  "http://192.168.178.21:81/otq-iaq.bin"

class Updater {
 public:
  // checks for update and gives new firmware URL
  static void check_for_update();

 private:
  // triggers the update process with a given firmware URL
  static void update();
};

#endif  // UPDATER_H
