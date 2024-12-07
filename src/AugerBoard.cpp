#include "AugerBoard.h"

void setup() {
    Serial.begin(115200);

    // Test Buttons
    pinMode(SW1,INPUT_PULLUP);
    pinMode(SW2,INPUT_PULLUP);
    pinMode(SW3,INPUT_PULLUP);

    // Direction Switch
    pinMode(DIR_SW, INPUT); 

    AugerAxis.attachEncoder(&AugerAxisEncoder);

    AugerAxis.attachHardLimits(&AugerAxisRVSLimit, &AugerAxisFWDLimit);

    AugerAxis.Motor()->configRampRate(5000);
    AugerMotor.configRampRate(5000);

    AugerAxisEncoder.begin([]{ AugerAxisEncoder.handleInterrupt(); });
    AugerEncoder.begin([]{ AugerAxisEncoder.handleInterrupt(); });

    AugerAxis.Motor()->configMaxOutputs(-1000,1000);
    AugerAxis.Motor()->configMinOutputs(0,0);

    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_AUGERBOARD_IPADDRESS);
    Telemetry.begin(telemetry,TELEMETRY_INTERVAL);
    Serial.println("Complete");

}

void loop() {

    RoveCommPacket packet = RoveComm.read();

    switch (packet.dataId){
        case RC_AUGERBOARD_AUGERAXIS_OPENLOOP_DATA_ID:
        {
            augerAxisDecipercent = *((int16_t*) packet.data);
            feedWatchDog();

            break;
        }
        case RC_AUGERBOARD_LIMITSWITCHOVERRIDE_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            AugerAxis.overrideForwardHardLimit(data & (1<<0));
            AugerAxis.overrideReverseHardLimit(data & (1<<1));

            break;
        }

        case RC_AUGERBOARD_REQUESTHUMIDITY_DATA_ID:
        {
            // uhh bunch of shit tbh

            break;
        }
        case RC_AUGERBOARD_AUGER_DATA_ID:
        {
            augerDecipercent = *((int16_t*) packet.data);
            feedWatchDog();

            break;
        }
        case RC_AUGERBOARD_WATCHDOGOVERRIDE_DATA_ID:
        {
            watchdogOverride = *((uint8_t*) packet.data);

            break;
        }
    }

    bool direction = digitalRead(DIR_SW);

    // AugerAxis
    if(!digitalRead(SW2)) AugerAxis.drive((direction ? -900 : 900));
    else AugerAxis.drive(augerAxisDecipercent);
    // Auger
    if(!digitalRead(SW1)) AugerAxis.drive((direction ? -900 : 900));
    else AugerMotor.drive(augerDecipercent);
    // Spare Motor
    if(!digitalRead(SW3)) SpareMotor.drive((direction ? -900 : 900));
    else SpareMotor.drive(0);
}


void telemetry(){

    // .write() vs .writeReliable()

    // Encoder position
    float position = AugerAxisEncoder.readDegrees();
    RoveComm.writeReliable(RC_AUGERBOARD_POSITION_DATA_ID,position);

    // Limit
    uint8_t limitSwitchValues = (AugerAxis.atForwardHardLimit()) | (AugerAxis.atReverseHardLimit());
    RoveComm.writeReliable(RC_AUGERBOARD_LIMITSWITCHTRIGGERED_DATA_ID,limitSwitchValues);

    // Watchdog status
    RoveComm.writeReliable(RC_AUGERBOARD_WATCHDOGSTATUS_DATA_ID,watchdogStatus);

}
float analogMap(){
    
}

void estop(){
    watchdogStatus = 1;
    if(!watchdogOverride){
        AugerAxis.drive(0);
        AugerMotor.drive(0); 

        augerAxisDecipercent = 0;
        augerDecipercent = 0;
    }

}

void feedWatchDog(){
    watchdogStatus = 0;
    Watchdog.begin(estop,WATCHDOG_TIMEOUT);
}

