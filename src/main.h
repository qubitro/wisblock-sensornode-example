#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <LoRaWan-RAK4630.h>

// Variables to be filles with info from TTN
#define DEV_EUI { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
#define APP_EUI { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
#define APP_KEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 60
#define LORAWAN_DATERATE DR_2
#define LORAWAN_TX_POWER TX_POWER_10
#define JOINREQ_NBTRIALS 3

// Comment out to disable a sensor
#define SHTC3_TEMP_HUM_SENSOR
#define LPS22_PRESSURE_SENSOR
#define OPT3001_LIGHT_SENSOR
#define LIS3DH_ACCEL_SENSOR
//#define BEM680_ENV_SENSOR
//#define UBLOX_GPS

// Comment out to disable powergating
#define POWERGATE_SENSORS

#define POLL_TIME 600000

#define OPT3001_ADDRESS 0x45

// Here since I don't have newest version of RAK's variant.h
#ifndef IO2 
#define POWERGATE_PIN 34
#else
#define POWERGATE_PIN IO2
#endif

#define PIN_VBAT WB_A0

#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER (0.4F)			  // 1.5M + 1M voltage divider on VBAT = (1.5M / (1M + 1.5M))
#define VBAT_DIVIDER_COMP (1.73)	  // (1.403F) // Compensation factor for the VBAT divider

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

// sensors.cpp
unsigned populateSensorData(uint8_t *buffer, unsigned len);
uint16_t readVBAT(void);
void initSensors();

// main.cpp lorawan handlers
static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);
static uint8_t GetBatteryLoRaWAN(void);
void tx_lora_periodic_handler(void);

#endif
