#include <Arduino.h>

struct NPKReading{
    uint16_t N;
    uint16_t P;
    uint16_t K;
};

class NPK{
    public:
        uint16_t NRead();
        uint16_t PRead();
        uint16_t KRead();
        NPKReading NPKRead();
        NPK(HardwareSerial *serial);
    private:
    // Define Modbus parameters
        const uint8_t slaveAddress = 0x01;          // Address of the Modbus slave device
        const uint8_t functionCode = 0x03;          // Function code to read holding registers
        const uint8_t startAddressHigh = 0x00;      // High uint8_t of the starting address
        const uint8_t startAddressLowN = 0x1E;      // Low uint8_t of teh starting address for N 
        const uint8_t startAddressLowP = 0x1F;      // Low uint8_t of the starting address for P
        const uint8_t startAddressLowK = 0x20;      // Low uint8_t of the starting address for K
        const uint8_t registerCountHigh = 0x00;     // High uint8_t of the number of registers to read
        const uint8_t registerCountLow = 0x01;      // Low uint8_t of the number of registers to read

        uint8_t requestFrame[8];
        uint8_t responseFrame[7];

        HardwareSerial *ModbusPort;
        NPKReading values;
        bool verifyCRC(uint8_t *frame, uint8_t length);
        uint16_t calculateCRC(uint8_t *frame, uint8_t length);
        uint16_t processModbusResponse(uint8_t *frame);
        void readModbusResponse(uint8_t *frame, uint8_t length);
        void sendModbusRequest(uint8_t *frame, uint8_t length);
        void constructModbusRequest(uint8_t *frame, uint8_t address, uint8_t function, uint8_t startHigh, uint8_t startLow, uint8_t countHigh, uint8_t countLow);
};