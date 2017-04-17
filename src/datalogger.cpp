#include "Arduino.h"
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <Wire.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "mroozon"
#define AIO_KEY         "xxx"


// Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);


const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void connect();

WiFiClient client;

const char MQTT_SERVER[] = AIO_SERVER;
const char MQTT_CLIENTID[] = AIO_KEY __DATE__ __TIME__;
const char MQTT_USERNAME[] = AIO_USERNAME;
const char MQTT_PASSWORD[] = AIO_KEY;

Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

const char TEMPERATURE_FEED[] = AIO_USERNAME "/feeds/temperatureX";
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);

const char AccX_FEED[] = AIO_USERNAME "/feeds/AccX";
Adafruit_MQTT_Publish AccX = Adafruit_MQTT_Publish(&mqtt, AccX_FEED);

void setup() {
    Serial.begin(9600);
    delay(10000);
    Wire.begin(2, 0);
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    Serial.println("Boot...");
    WiFiManager wifiManager;
    wifiManager.autoConnect("AutoConnectAP");
    connect();
}

void loop() {

    // ping adafruit io a few times to make sure we remain connected
    if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
    }

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    Serial.print("AcX = "); Serial.print(AcX);
    // Serial.print(" | AcY = "); Serial.print(AcY);
    // Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Tmp = "); Serial.println(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    // Serial.print(" | GyX = "); Serial.print(GyX);
    // Serial.print(" | GyY = "); Serial.print(GyY);
    // Serial.print(" | GyZ = "); Serial.println(GyZ);

    // Publish data
    if (! temperature.publish(Tmp/340.00+36.53))
      Serial.println(F("Failed to publish temperature"));
    else
      Serial.println(F("Temperature published!"));
    if (! AccX.publish(AcX))
      Serial.println(F("Failed to publish AccX"));
    else
      Serial.println(F("AccX published!"));

     // Repeat every 10 seconds
     delay(10000);

    }
    // connect to adafruit io via MQTT
    void connect() {

      Serial.print(F("Connecting to Adafruit IO... "));

      int8_t ret;

      while ((ret = mqtt.connect()) != 0) {

        switch (ret) {
          case 1: Serial.println(F("Wrong protocol")); break;
          case 2: Serial.println(F("ID rejected")); break;
          case 3: Serial.println(F("Server unavail")); break;
          case 4: Serial.println(F("Bad user/pass")); break;
          case 5: Serial.println(F("Not authed")); break;
          case 6: Serial.println(F("Failed to subscribe")); break;
          default: Serial.println(F("Connection failed")); break;
        }

        if(ret >= 0)
          mqtt.disconnect();

        Serial.println(F("Retrying connection..."));
        delay(5000);

      }

      Serial.println(F("Adafruit IO Connected!"));

    }
