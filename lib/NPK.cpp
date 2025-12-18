#include <Arduino.h>
#include "NPK.h"

// Create a SoftwareSerial object to communicate with the MAX485 module
NPK::NPK(HardwareSerial *serial){
    ModbusPort=serial;
}

bool NPK::verifyCRC(uint8_t *frame, uint8_t length) {
    uint16_t receivedCRC = (frame[length - 1] << 8) | frame[length - 2]; // Extract the received CRC
    // Calculate the CRC of the received frame (excluding the received CRC uint8_ts)
    return calculateCRC(frame, length - 2) == receivedCRC;
}

uint16_t NPK::calculateCRC(uint8_t *frame, uint8_t length) {
    uint16_t crc = 0xFFFF; // Initialize CRC to 0xFFFF
    for (uint8_t i = 0; i < length; i++) {
        crc ^= frame[i]; // XOR the frame uint8_t with the CRC
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) { // Check if the LSB of the CRC is 1
                crc >>= 1;        // Right shift the CRC
                crc ^= 0xA001;    // XOR the CRC with the polynomial 0xA001
            } else {
                crc >>= 1;        // Right shift the CRC
            }
        }
    }
    return crc; // Return the calculated CRC
}

uint16_t NPK::processModbusResponse(uint8_t *frame) {
    // Extract the humidity and temperature data from the response frame
    uint16_t Value = (frame[3] << 8) | frame[4];
    return Value;
}

// Function to read a Modbus response frame
void NPK::readModbusResponse(uint8_t *frame, uint8_t length) {
    uint8_t readLength;
    if (ModbusPort->available()) {
        readLength = ModbusPort->readBytes(frame, length); // Read each uint8_t of the frame
    }else {
        Serial.print("unavailable");
    }
}

// Function to send a Modbus request frame
void NPK::sendModbusRequest(uint8_t *frame, uint8_t length) {
  ModbusPort->write(frame, length); // Send each uint8_t of the frame 
}

// Function to construct a Modbus request frame
void NPK::constructModbusRequest(uint8_t *frame, uint8_t address, uint8_t function, uint8_t startHigh, uint8_t startLow, uint8_t countHigh, uint8_t countLow) {
  frame[0] = address;          // Address of the slave device
  frame[1] = function;         // Function code
  frame[2] = startHigh;        // High uint8_t of the starting address
  frame[3] = startLow;         // Low uint8_t of the starting address
  frame[4] = countHigh;        // High uint8_t of the number of registers to read
  frame[5] = countLow;         // Low uint8_t of the number of registers to read

  // Calculate and append the CRC to the request frame
  uint16_t crc = calculateCRC(frame, 6);
  frame[6] = crc & 0xFF;         // CRC low uint8_t
  frame[7] = (crc >> 8) & 0xFF;  // CRC high uint8_t
}

uint16_t NPK::NRead(){
  constructModbusRequest(requestFrame, slaveAddress, functionCode, startAddressHigh, startAddressLowN, registerCountHigh, registerCountLow);

  // Send the Modbus request frame
  sendModbusRequest(requestFrame, 8);

  // Read and process the Modbus response frame
  if (ModbusPort->available()) {
    // Read the response frame from the slave device
    readModbusResponse(responseFrame, 7);
    // Verify the CRC of the received response frame
    if (verifyCRC(responseFrame, 7)) {
      // Process the response frame to extract data
      return processModbusResponse(responseFrame);
    } else {
        
    }
  } else {
  }
}

uint16_t NPK::PRead(){
  constructModbusRequest(requestFrame, slaveAddress, functionCode, startAddressHigh, startAddressLowP, registerCountHigh, registerCountLow);

  // Send the Modbus request frame
  sendModbusRequest(requestFrame, 8);

  // Read and process the Modbus response frame
  if (ModbusPort->available()) {
    // Read the response frame from the slave device
    readModbusResponse(responseFrame, 7);
    // Verify the CRC of the received response frame
    if (verifyCRC(responseFrame, 7)) {
      // Process the response frame to extract data
      return processModbusResponse(responseFrame);
    } else {
    }
  } else {
  }
}

uint16_t NPK::KRead(){
  constructModbusRequest(requestFrame, slaveAddress, functionCode, startAddressHigh, startAddressLowK, registerCountHigh, registerCountLow);

  // Send the Modbus request frame
  sendModbusRequest(requestFrame, 8);

  // Read and process the Modbus response frame
  if (ModbusPort->available()) {
    // Read the response frame from the slave device
    readModbusResponse(responseFrame, 7);
    // Verify the CRC of the received response frame
    if (verifyCRC(responseFrame, 7)) {
      // Process the response frame to extract data
      return processModbusResponse(responseFrame);
    } else {
    }
  } else {
  }
}

NPKReading NPK::NPKRead(){
    NPKReading vals;
    vals.N = NRead();
    vals.P = PRead();
    vals.K = KRead();
}