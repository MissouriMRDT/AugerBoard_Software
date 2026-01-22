#ifndef AUGERBOARD_H
#define AUGERBOARD_H

#include "PinAssignments.h"
#include <PWMServo.h>
#include <RoveComm.h>
#include <arduino.h>

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

#endif
