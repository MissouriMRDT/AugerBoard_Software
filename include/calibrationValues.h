#ifndef CALIBRATIONVALUES_H
#define CALIBRATIONVALUES_H

// These values are used to calibrate the humidity sensor readings. 
// the testmV values are the raw ADC readings from the sensor at the labeled percent GWC
// actual humidity values were found using the procedure outlined in the moisture sensor calibration document
// This can be found in arch science -> documentation -> calibration

uint16_t testmV0 = 729;
float actualhumidity0 = 0.0f;
uint16_t testmV10 = 729;
float actualhumidity10 = 0.0f;
uint16_t testmV20 = 530;
float actualhumidity20 = 24.3f;
uint16_t testmV30 = 500;
float actualhumidity30 = 36.06f;
uint16_t testmV40 = 433;
float actualhumidity40 = 47.29f;
uint16_t testmV50 = 377;
float actualhumidity50 = 52.25f;


#endif