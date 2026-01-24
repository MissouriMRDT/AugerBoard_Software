#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H

// Servo Pins
#define SOIL_TRAPDOOR_PWM 5
#define SCI_GIMBAL_TILT 6
#define SCI_GIMBAL_PAN 24
#define SOIL_CACHE_SPARE_PWM 28
#define AF_SPARE_PWM 29
#define AF_LENS_PWM 12

// LED Pins
#define GIMBAL_TILT_LED 7
#define AF_SPARE_LED 30
#define SOIL_TD_LED 4
#define VESC_LED 22
#define SPARE_MOTOR_LED 2
#define AFF_LED 11
#define CACHE_SPARE_LED 26
#define GIMBAL_PAN_LED 25
#define GANTRY_LED 23

// Button Pins
#define GIMBAL_TILT_SW 34
#define AF_SPARE_SW 35
#define DIR_SW 33
#define S_CACHE_SPARE_SW 16
#define SOIL_DOOR_SW 17
#define GANTRY_SW 18
#define GIMBAL_PAN_SW 37
#define SPARE_SW_12V 36
#define AUGER_SW 40
#define AF_LENS_SW 39

// Autoflourescence Pins
#define AF_LED_365 8
#define AF_LED_405 9
#define AF_LED_500 10
#define AF_WHITE_LED 31

// Temperature and Humidity Sensor Pins
#define TEMP 38
#define MOISTURE 27

// CAN Signal Pins
#define RXD_1 0
#define TXD_1 1
#define STBY_1 3
#define RXD_2 21
#define TXD_2 20
#define STBY_2 19

// NPK and pH Sensor Pins
#define RO 41
#define RE 13
#define DE 14
#define DI 15

#endif
