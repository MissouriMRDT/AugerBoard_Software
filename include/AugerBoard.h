#ifndef AUGERBOARD_H
#define AUGERBOARD_H

#include "PinAssignments.h"
#include <PWMServo.h>
#include <RoveComm.h>
#include <arduino.h>
#include <LimitSwitch.h>
#include <Smoco.h>

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
int16_t augerAxisDecipercent = 0;


// Limit Switches



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
    

// Autoflouresence

// Servos
#define SOIL_CACHE_ANGLE 0
#define AF_LENS_ANGLE 0
#define GIMBAL_PAN_ANGLE 0
#define GIMBAL_TILT_ANGLE 0
PWMServo soilTrapdoor;
PWMServo AFLens;
PWMServo gimbalPan;
PWMServo gimbalTilt;
int16_t soilTrapdoorAngle = 0;
int16_t AFLensAngle = 0;
int16_t gimbalPanAngle = 0;
int16_t gimbalTiltAngle = 0;
uint32_t lastServoUpdate = 0;


// Functions
void estop();
void telemetry();
void feedWatchdog();
float readTemperature();
float readHumidity();



#endif
