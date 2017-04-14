#include "Arduino.h"

#include <OneWire.h>
#include <DallasTemperature.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>


void setup() {
    Serial.begin(9600);
    delay(10000);
    Serial.println("Boot...");
    WiFiManager wifiManager;
    wifiManager.autoConnect("AutoConnectAP");
}

void loop() {
    delay(1000);
    Serial.println("Loop...");
}
