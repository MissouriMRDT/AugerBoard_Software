#ifndef AUGERBOARD_H
#define AUGERBOARD_H

#include "PinAssignments.h"
#include <LimitSwitch.h>
#include <PWMServo.h>
#include <RoveComm.h>
#include <Smoco.h>
#include <arduino.h>

// RoveComm
RoveCommEthernet RoveComm;

// Watchdog
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;
uint8_t watchdogStatus = 0;
uint8_t watchdogOverride = 0;

#define TELEMETRY_INTERVAL 1000
IntervalTimer Telemetry;

// Motors
int16_t augerAxisDecipercent = 0;

// Limit Switches

// Analog Mapping Function
float analogMap(uint16_t measurement, uint16_t fromADC, uint16_t toADC, float fromAnalog, float toAnalong);
float calibratedAnalogMapHumidity(int measurement);
float calibratedAnalogMapTemp(int measurement);
int veryWet = 0;
int middleWet = 512;
int veryDry = 1023;
int veryCold = 0;
int middleCold = 512;
int veryWarm = 1023;

// CAN Setup
// TO DO: Add ability to switch can lines using switch
// or set up both simultaniously
#define auger_axis_can ACAN_T4::can1
// #define auger_motor_can ACAN_T4::can2
ACAN_T4_Settings canSettings(125 * 1000);
Smoco augerGantry(&auger_axis_can, 0x00);
uint32_t pingTime = 0;

// Will be VESC but also CAN?
// Smoco auger_motor(&auger_motor_can, 0x01);

// LEDs
#define LED_DURATION 500

// Servos
#define SOIL_CACHE_ANGLE_LEFT 45
#define SOIL_CACHE_ANGLE_RIGHT 135
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
int tempCelcius;
float temperature = 0;
float humidity = 0;
uint32_t lastTempRead = 0;
uint32_t lastHumidityRead = 0;
uint8_t dataCountTemp = 0;
uint8_t dataCountHumidity = 0;
uint16_t avgTemp = 0;
uint16_t avgHumidity = 0;

// Functions
void estop();
void telemetry();
void feedWatchdog();

#endif
