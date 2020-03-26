#include <updater.h>

StaticJsonDocument<400> static_json_document;

HTTPClient updater_http_client;

bool isUpdating = false;

void Updater::check_for_update() {
  Serial.printf("[Updater] check for update %s\n", UPDATER_VERSION_URL);
  updater_http_client.begin(UPDATER_VERSION_URL);
  int httpCode = updater_http_client.GET();
  if (httpCode == HTTP_CODE_OK) {
    Serial.println("[Updater] Version request succeeded");
    DeserializationError deserializationError =
        deserializeJson(static_json_document, updater_http_client.getString());
    if (deserializationError) {
      Serial.printf("[Updater] could not deserialize json");
      updater_http_client.end();
      static_json_document.clear();
    } else {
      const char *remote_version =
          static_json_document["version"].as<const char *>();
      Serial.printf("[Updater] remote version %s\n", remote_version);
      bool update_needed = !String(remote_version).equals(FIRMWARE_VERSION);
      updater_http_client.end();
      static_json_document.clear();
      if (update_needed) {
        Serial.printf("[Updater] %s is not equals %s\n", remote_version,
                      FIRMWARE_VERSION);
        Updater::update();
      } else {
        Serial.printf("[Updater] Latest Firmware is installed!\n");
      }
    }
  } else {
    Serial.printf("[Updater] Could not check FIRMWARE_VERSION code %d\n",
                  httpCode);
    updater_http_client.end();
  }
}

void Updater::update() {
  isUpdating = true;
  Serial.printf("[Updater] Updating from %s\n", UPDATER_FIRMWARE_URL);
  updater_http_client.begin(UPDATER_FIRMWARE_URL);

  Serial.println("[Updater] start query ...");
  int result_code = updater_http_client.GET();
  if (result_code > 0) {
    Serial.printf("[Updater] result code %d\n", result_code);
    if (result_code == HTTP_CODE_OK) {
      // check payload byte size
      long payload_bytes = updater_http_client.getSize();
      if (payload_bytes > 0) {
        Serial.println("[Updater] start updating ... (this may takes a while)");
        Update.begin(payload_bytes);
        updater_http_client.getStreamPtr()->setTimeout(1000);
        int written_bytes =
            Update.writeStream(*updater_http_client.getStreamPtr());
        updater_http_client.end();
        if (written_bytes == payload_bytes) {
          if (Update.end()) {
            Serial.println("[Updater] OTA done!");
            if (Update.isFinished()) {
              Serial.println(
                  "[Updater] Update successfully completed. Rebooting.");
            } else {
              Serial.println(
                  "[Updater] Update not finished? Something went wrong!");
            }
          } else {
            Serial.println("[Updater] Error Occurred. Error #: " +
                           String(Update.getError()));
          }
        } else {
          Serial.printf("[Updater] update failed, only %d bytes written\n",
                        written_bytes);
        }
      } else {
        updater_http_client.end();
        Serial.printf("[Updater] zero bytes received\n");
      }
    } else {
      updater_http_client.end();
      Serial.printf("[Updater] query failed because of code %d\n", result_code);
    }
  } else {
    updater_http_client.end();
    Serial.println("[Updater] query failed");
  }

  Serial.printf("[Updater] update done...");
  Serial.flush();
  WiFi.disconnect();
  ESP.restart();

}