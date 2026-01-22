#include "AugerBoard.h"

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);

    // Test Buttons
    pinMode(GIMBAL_TILT_SW, INPUT_PULLUP);
    pinMode(AF_SPARE_SW, INPUT_PULLUP);
    pinMode(S_CACHE_SPARE_SW, INPUT_PULLUP);
    pinMode(SOIL_DOOR_SW, INPUT_PULLUP);
    pinMode(GANTRY_SW, INPUT_PULLUP);
    pinMode(GIMBAL_PAN_SW, INPUT_PULLUP);
    pinMode(SPARE_SW_12V, INPUT_PULLUP);
    pinMode(AUGER_SW, INPUT_PULLUP);
    pinMode(AF_LENS_SW, INPUT_PULLUP);

    // Direction Switch
    pinMode(DIR_SW, INPUT);

    // Sensors
    pinMode(TEMP, INPUT);
    pinMode(MOISTURE, INPUT);

    // Autoflouresence
    pinMode(AF_LED_365, OUTPUT);
    pinMode(AF_LED_405, OUTPUT);
    pinMode(AF_LED_490, OUTPUT);
    pinMode(AF_WHITE_LED, OUTPUT);

    RoveComm.begin(RC_AUGERBOARD_IPADDRESS);
}

RoveCommPacket packet;
void loop() {
    RoveComm.read(packet);

    switch (packet.dataId) {
    case RC_AUGERBOARD_AUGERAXIS_DATA_ID: {
        break;
    }
    case RC_AUGERBOARD_LIMITSWITCHOVERRIDE_DATA_ID: {
        break;
    }
    case RC_AUGERBOARD_CALIBRATEENCODER_DATA_ID: {
        break;
    }
    case RC_AUGERBOARD_AUGER_DATA_ID: {
        break;
    }
    case RC_AUGERBOARD_WATCHDOGOVERRIDE_DATA_ID: {
        break;
    }
    case RC_AUGERBOARD_LED_DATA_ID: {
        break;
    }
    case RC_AUGERBOARD_AUGERSERVO_DATA_ID: {
        break;
    }
    case RC_AUGERBOARD_AUGERGIMBAL_DATA_ID: {
        break;
    }

    }
}
