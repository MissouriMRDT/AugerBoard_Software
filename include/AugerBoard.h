#ifndef AUGERBOARD_H
#define AUGERBOARD_H

#include "PinAssignments.h"
#include "calibrationValues.h"
#include <Bounce.h>
#include <LimitSwitch.h>
#include <PWMServo.h>
#include <RoveComm.h>
#include <Smoco.h>
#include <algorithm>
#include <vesc_can_sdk.h>

// RoveComm
RoveCommEthernet RoveComm;

// Watchdog
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;
uint8_t watchdogStatus = 0;
uint8_t watchdogOverride = 0;

// Telemetry
#define TELEMETRY_INTERVAL 500000
IntervalTimer Telemetry;
uint32_t lastPrint = 0;

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

// Gantry Motor
Smoco augerGantry(&auger_axis_can, 8);
int16_t augerAxisDecipercent = 0;
Bounce gantryButton(GANTRY_SW, 50);
const double INCHES_PER_STEP = 2.61077e-4;

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
float calibratedAnalogMapHumidity(uint16_t measurement);

// LEDs
#define LED_DURATION 100 // ms

// Servos
#define AF_LENS_ANGLE_GREEN 90
#define AF_LENS_ANGLE_RED 155
#define AF_LENS_ANGLE_BLUE 18
#define AF_LENS_ANGLE_BLANK 180 // Just for reference
PWMServo soilTrapdoor;
PWMServo AFLens;
PWMServo gimbalPan;
PWMServo gimbalTilt;
int16_t soilTrapdoorAngle = 0;
int16_t AFLensAngle = 0;
int16_t gimbalPanAngle = 90;
int16_t gimbalTiltAngle = 90;
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
uint32_t lastTempRead = 0;
uint32_t lastHumidityRead = 0;
int humidityReading;
uint8_t dataCountTemp = 0;
uint8_t dataCountHumidity = 0;
bool hasFilledArrayTemp = false;
bool hasFilledArrayHumidity = false;
float averageReadingTemp;
float averageReadingHumidity;
float readingsHumidity[10];
uint16_t humidity;

#define AREF 3.3f
#define ADC_RESOLUTION 10
float readingTemp, voltage, temperature;
float readingsTemp[10];

float get_voltage(int raw_adc);
float get_temperature(float voltage);

// Functions
void estop();
void telemetry();
void feedWatchdog();

// AF LED Shutdown Logic
bool LEDs_on = false;
bool LEDWatchdog = false;
uint32_t WatchdogTimer = 0;
uint32_t LEDStartTime = 0;
uint32_t LEDTimer = 0;
uint32_t LEDStoppedTime = 0;
bool LEDStopped = false;

#endif
