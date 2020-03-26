# ESP-32 firmware for indoor air quality with bme680



Uses an ESP-32 (Heltec Board with OLED Display) to measure indoor air quality and send results to MQTT broker
Features
* handles bme680 bosch BESC lib (incl. EEPROM)
* display current air quality on display
* send all data via mqtt 

Build by setting the wifi SSID and passwort via

export PLATFORMIO_BUILD_FLAGS="'-DWSSID=\"yourSSID\"' '-DWPWD=\"mypwd\"'"

![](images/IMG_20200221_202054.jpg)