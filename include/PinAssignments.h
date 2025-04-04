#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H

// MoCo Pins        (IN_A == HIGH) && (IN_B == LOW) -> Clockwise
//                  IN_A == fwdPin, IN_B == rvsPin

#define IN_B1   9
#define PWM1    15
#define CS1     14
#define IN_A1   12

#define IN_A2   41
#define CS2     27
#define PWM2    24
#define IN_B2   32

// Auger VESC
#define AUGER_SERIAL Serial2

#define RX      7 
#define TX      8
// #define PWM3    24
// #define IN_B3   39

// Encoder Pins
#define ENCODER_1A 26
#define ENCODER_1B 25

// Limit Switch Pins
#define LIMITSWITCH1 6  // Top left
#define LIMITSWITCH2 10 // Top Right
#define LIMITSWITCH3 4  // Bottom Left
// #define LIMITSWITCH4  //

// Motor Button Pins
#define SW1 16 // Auger
#define SW2 13// Auger Gantry
#define SW3 35 // Spare

// Direction Switch Pin
#define DIR_SW 37

// Temperature and Humidity Pins
#define TEMP     38
#define HUMIDITY 17
#define UVLED    36 // Magwire

#define SERVO_PWM1 29
#define SERVO_PWM2 33

#endif
