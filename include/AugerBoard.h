#ifndef AUGERBOARD_H
#define AUGERBOARD_H

#include "PinAssignments.h"
#include <Bounce.h>
#include <LimitSwitch.h>
#include <PWMServo.h>
#include <RoveComm.h>
#include <Smoco.h>
#include <arduino.h>
#include <vesc_can_sdk.h>

// RoveComm
RoveCommEthernet RoveComm;

// Watchdog
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;
uint8_t watchdogStatus = 0;
uint8_t watchdogOverride = 0;

// Telemetry
#define TELEMETRY_INTERVAL 1000000
IntervalTimer Telemetry;

// Gantry Motor
int16_t augerAxisDecipercent = 0;
Bounce gantryButton(GANTRY_SW, 50);

// Auger Motor
#define VESC_ID 115
bool send_msg(uint32_t id, uint8_t *data, uint8_t len);
void response_callback(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len);
float dutyCycle = 0.0f;
float augerSpeed = 0.0f;
float augerCurrent = 0.0f;
Bounce augerButton(AUGER_SW, 50);

// Analog Mapping Function
float analogMap(uint16_t measurement, uint16_t fromADC, uint16_t toADC, float fromAnalog, float toAnalong);
float calibratedAnalogMapHumidity(int measurement);
float calibratedAnalogMapTemp(int measurement);
int veryWet = 380;
int veryDry = 765;
int veryCold = 0;
int middleCold = 512;
int veryWarm = 1023;

// CAN Setup
#define USE_CAN2 1

#if USE_CAN2
#define auger_axis_can ACAN_T4::can2
#define auger_motor_can ACAN_T4::can2
#else
#define auger_axis_can ACAN_T4::can1
#define auger_motor_can ACAN_T4::can1
#endif
void process_can_message();
ACAN_T4_Settings canSettings(125 * 1000);
Smoco augerGantry(&auger_axis_can, 8);

// LEDs
#define LED_DURATION 100

// Servos
#define SOIL_CACHE_ANGLE_LEFT -15
#define SOIL_CACHE_ANGLE_RIGHT 153
#define AF_LENS_ANGLE_1 45
#define AF_LENS_ANGLE_2 90
#define AF_LENS_ANGLE_3 135
#define AF_LENS_ANGLE_BLANK 180
PWMServo soilTrapdoor;
PWMServo AFLens;
PWMServo gimbalPan;
PWMServo gimbalTilt;
int16_t soilTrapdoorAngle = 0;
int16_t AFLensAngle = 0;
int16_t gimbalPanAngle = 0;
int16_t gimbalTiltAngle = 0;
uint32_t lastServoUpdate = 0;
uint32_t lastSoilTrapdoorUpdate = 0;
uint32_t lastAFLensUpdate = 0;
uint32_t lastGimbalPanUpdate = 0;
uint32_t lastGimbalTiltUpdate = 0;
bool isSoilTrapdoorMoving = false;
bool isAFLensMoving = false;
bool isGimbalPanMoving = false;
bool isGimbalTiltMoving = false;

// Temperature and Humidity Sensors
float tempCelcius = 0;
float temperatureSum = 0;
float humidity = 0;
float humiditySum = 0;
uint32_t lastTempRead = 0;
uint32_t lastHumidityRead = 0;
uint8_t dataCountTemp = 0;
uint8_t dataCountHumidity = 0;
float avgTemp = 0.0f;
float avgHumidity = 0.0f;

// Functions
void estop();
void telemetry();
void feedWatchdog();

#endif
