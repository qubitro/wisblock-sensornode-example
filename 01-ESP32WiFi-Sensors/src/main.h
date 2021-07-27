#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <QubitroMqttClient.h>
#include <WiFi.h> 
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

// Comment out to disable a sensor
#define SHTC3_TEMP_HUM_SENSOR
#define LPS22_PRESSURE_SENSOR
//#define OPT3001_LIGHT_SENSOR
//#define LIS3DH_ACCEL_SENSOR
//#define BEM680_ENV_SENSOR
//#define UBLOX_GPS

#define VBAT_MV_PER_LSB 10
#define VBAT_DIVIDER_COMP 0
#define PIN_VBAT WB_A0
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

// Comment out to disable powergating
#define POWERGATE_SENSORS

#define POLL_TIME 600000

#define USE_SSL 0

#define BUFFER_LEN 1024

bool populateSensorData(char *buffer);
void initSensors();

#endif