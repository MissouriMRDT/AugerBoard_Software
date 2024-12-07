#ifndef AUGERBOARD_H
#define AUGERBOARD_H

#include <Arduino.h>
#include "PinAssignments.h"
#include <RoveComm.h>
#include <RoveMotor.h>
#include <RoveQuadEncoder.h>
#include <RoveJoint.h>
#include <LimitSwitch.h>
#include <RoveVNH.h>


//temp and humidity max min shit and dividers idk hai

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

RoveVNH AugerAxisMotor(PWM1,IN_A1,IN_B1,CS1);
RoveVNH AugerMotor(PWM2,IN_A2,IN_B2,CS2);
RoveVNH SpareMotor(PWM3,IN_A3,IN_B3,CS3);

// Encoders
RoveQuadEncoder AugerAxisEncoder(ENCODER_2A , ENCODER_2B, 360);
RoveQuadEncoder AugerEncoder(ENCODER_1A, ENCODER_1B, 360);


// Limit Switches
LimitSwitch AugerAxisRVSLimit(LIMITSWITCH1);
LimitSwitch AugerAxisFWDLimit(LIMITSWITCH2);
LimitSwitch AugerAxisRVSLimitSpare(LIMITSWITCH3);
LimitSwitch AugerAxisFWDLimitSpare(LIMITSWITCH4);

// Joints
RoveJoint AugerAxis(&AugerAxisMotor);


// Control variables
int16_t augerDecipercent = 0;
int16_t augerAxisDecipercent = 0;

// Methods
// analogMap function eurm
float analogMap(uint16_t measurement, uint16_t fromADC, uint16_t toADC, float fromAnalog, float toAnalog);
void telemetry();
void feedWatchdog();
void estop();


// this is rlly awkward :grimacing:
#define RC_AUGERBOARD_IPADDRESS RC_SCIENCEACTUATIONBOARD_IPADDRESS

#define RC_AUGERBOARD_LIMITSWITCHOVERRIDE_DATA_ID RC_SCIENCEACTUATIONBOARD_LIMITSWITCHOVERRIDE_DATA_ID
#define RC_AUGERBOARD_AUGER_DATA_ID RC_SCIENCEACTUATIONBOARD_AUGER_DATA_ID
#define RC_AUGERBOARD_REQUESTHUMIDITY_DATA_ID RC_SCIENCEACTUATIONBOARD_REQUESTHUMIDITY_DATA_ID
#define RC_AUGERBOARD_WATCHDOGOVERRIDE_DATA_ID RC_SCIENCEACTUATIONBOARD_WATCHDOGOVERRIDE_DATA_ID

#define RC_AUGERBOARD_POSITION_DATA_ID 69
#define RC_AUGERBOARD_POSITION_DATA_COUNT 1
#define RC_AUGERBOARD_POSITION_DATA_TYPE float

#define RC_AUGERBOARD_WATCHDOGSTATUS_DATA_ID 420

#define RC_AUGERBOARD_LIMITSWITCHTRIGGERED_DATA_ID 123456789

#define RC_AUGERBOARD_AUGERAXIS_OPENLOOP_DATA_ID 987654321
#endif
