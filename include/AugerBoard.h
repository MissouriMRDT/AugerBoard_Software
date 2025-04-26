#ifndef AUGERBOARD_H
#define AUGERBOARD_H

#include "PinAssignments.h"
#include <Arduino.h>
#include <LimitSwitch.h>
#include <RoveComm.h>
#include <RoveJoint.h>
#include <RoveMotor.h>
#include <RoveQuadEncoder.h>
#include <RoveVNH.h>
#include <RoveVESC.h>
#include <PWMServo.h>

// Temp
int tempCelcius;

// Define Humidity Sensor Calibration Variables
int veryWet = 374;
int veryDry = 764;
int humidityRange = veryWet - veryDry;

// RoveComm
RoveCommEthernet RoveComm;

// Watchdog
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;
uint8_t watchdogStatus = 0;
uint8_t watchdogOverride = 0;

#define TELEMETRY_INTERVAL 300000
IntervalTimer Telemetry;

// Motors
// auger vesc
RoveVESC AugerMotor(&AUGER_SERIAL);
RoveVNH AugerAxisMotor(PWM2, IN_A2, IN_B2, CS2);
RoveVNH SpareMotor(PWM1, IN_A1, IN_B1, CS1);

// Encoders
RoveQuadEncoder AugerAxisEncoder(ENCODER_1A, ENCODER_1B, 100000 / 360.0f);

// Limit Switches
LimitSwitch AugerAxisRVSLimit(LIMITSWITCH2);
LimitSwitch AugerAxisFWDLimit(LIMITSWITCH3);
LimitSwitch AugerAxisRVSLimitSpare(LIMITSWITCH1); //uhh


// Joints
// vesc
RoveJoint AugerAxis(&AugerAxisMotor);

// Servos
PWMServo multiplexer; //, CachedServo servo2();  eurm

// Control variables
int16_t augerDecipercent = 0; //
int16_t augerAxisDecipercent = 0;
int16_t multiplexerAngle = 0;
uint32_t lastMultiplexerUpdate = 0;

// Sensors
float temperature = 0; // degrees C
float humidity = 0;
float readTemperature();
float readHumidity();

// Methods
// analogMap function eurm
float analogMap(uint16_t measurement, uint16_t fromADC, uint16_t toADC, float fromAnalog, float toAnalog);
void telemetry();
void feedWatchdog();

// Autofluorescence
#define MAX_UVLED_LEVEL 500
// Timeout after 5 seconds to prevent thermal runaway
#define MAX_UVLED_ON_PERIOD 5'000'000
IntervalTimer UVLEDWatchdog;
void enableUVLED(bool enable);
void estopUVLED();

void estop();

#endif
