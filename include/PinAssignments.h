#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H

// MoCo Pins        // (IN_A1 == HIGH) & (IN_B1 == LOW) == Clockwise
#define IN_B1   33
#define PWM1    23
#define CS1     38
#define IN_A1   4

#define IN_A2   31
#define CS2     26
#define PWM2    25
#define IN_B2   28

#define IN_A3   40
#define CS3     27
#define PWM3    24
#define IN_B3   39

// Encoder Pins
#define ENCODER_1A 14
#define ENCODER_1B 15
#define ENCODER_2A 16
#define ENCODER_2B 17

// Limit Switch Pins
#define LIMITSWITCH1 0 // uhhhhhhhhhhhhhhh
#define LIMITSWITCH2 1 //
#define LIMITSWITCH3 7 //
#define LIMITSWITCH4 8 //

// Motor Button Pins
#define SW1 29 // Auger
#define SW2 34 // Auger Gantry
#define SW3 35 // Spare erm

// Direction Switch Pin
#define DIR_SW 18

// Temperature and Humidity Pins
#define TEMP     21
#define HUMIDITY 22

// IR Receiver Pin(s?)
#define VIR 20

#endif
