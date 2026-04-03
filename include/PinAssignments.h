#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H

// Servo Pins
#define SOIL_TRAPDOOR_PWM 29
#define SCI_GIMBAL_TILT 11
#define SCI_GIMBAL_PAN 12
#define SPARE_PWM_1 9
#define SPARE_PWM_2 8
#define AF_LENS_PWM 28

// LED Pins
#define GIMBAL_TILT_LED 10
#define SPARE_LED_1 7
#define SOIL_TD_LED 30
#define VESC_LED 34
#define SPARE_12V_LED 13
#define AFF_LED 31
#define SPARE_LED_2 6
#define GIMBAL_PAN_LED 5
#define GANTRY_LED 33

// Button Pins
#define GIMBAL_TILT_SW 15
#define SPARE_SW_1 17
#define DIR_SW 35
#define SPARE_SW_2 18
#define SOIL_DOOR_SW 14
#define GANTRY_SW 41
#define GIMBAL_PAN_SW 40
#define SPARE_SW_12V 16
#define AUGER_SW 36
#define AF_LENS_SW 37

// Autoflourescence Pins
#define AF_LED_365 25
#define AF_LED_405 19
#define AF_LED_500 3
#define AF_WHITE_LED 24

// Temperature and Humidity Sensor Pins
#define TEMP 26
#define MOISTURE 27

// CAN Signal Pins
#define RXD_1 23
#define TXD_1 22
#define STBY_1 4
#define RXD_2 0
#define TXD_2 1
#define STBY_2 2

#endif
