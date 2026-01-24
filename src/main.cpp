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
    pinMode(AF_LED_500, OUTPUT);
    pinMode(AF_WHITE_LED, OUTPUT);
    analogWrite(AF_WHITE_LED, 0);
    analogWrite(AF_LED_365, 0);
    analogWrite(AF_LED_405, 0);
    analogWrite(AF_LED_500, 0);

    // Servos
    soilTrapdoor.attach(SOIL_TRAPDOOR_PWM);
    AFLens.attach(AF_LENS_PWM);
    gimbalPan.attach(SCI_GIMBAL_PAN);
    gimbalTilt.attach(SCI_GIMBAL_TILT);

    RoveComm.begin(RC_AUGERBOARD_IPADDRESS);
}

RoveCommPacket packet;
void loop() {

    augerGantry.setLowPassSmoothingFactor(UINT16_MAX);
    augerGantry.setPID(1, 0, 0);
    augerGantry.setSoftLimitPosition(INT32_MIN, INT32_MAX);

    augerGantry.m_lowPassSmoothingAlpha = UINT16_MAX;
    augerGantry.m_PID = (1, 0, 0);
    // augerGantry.m_positionA = INT32_MIN;

    RoveComm.read(packet);

    switch (packet.dataId) {
    case RC_AUGERBOARD_AUGERAXIS_DATA_ID: {
        // sends a speed percentage to a smoco controlling the auger gantry
        // gantry.openLoop(())
        augerGantry.openLoopDrive(packet.data, augerGantry.m_ignoreLimit);
        feedWatchdog();

        break;
    }
    case RC_AUGERBOARD_LIMITSWITCHOVERRIDE_DATA_ID: {
        // sets auger axis motor object limit switch overide to be true for both FWD & REV
        uint8_t data = *(uint8_t *)packet.data;
        augerGantry.m_ignoreLimit = (data & (1 << 0)) || (data & (1 << 1));

        break;
    }
    case RC_AUGERBOARD_CALIBRATEENCODER_DATA_ID: {
        // sends a command to smoco to drive gantry motor up until it triggers a limit switch, and sets that point to
        // zero for the encoder
        augerGantry.startPositionCalibration((INT16_MIN / 4), 0);

        break;
    }
    case RC_AUGERBOARD_AUGER_DATA_ID: {
        // sets the speed for the auger motor
        feedWatchdog();

        break;
    }
    case RC_AUGERBOARD_WATCHDOGOVERRIDE_DATA_ID: {
        // disables the watchdog interrupt
        watchdogOverride = *((uint8_t *)packet.data);

        break;
    }
    case RC_AUGERBOARD_LED_DATA_ID: {
        // Sends PWM signal to AF LED for brightness & type

        analogWrite(AF_WHITE_LED, packet.data[0]);
        analogWrite(AF_LED_365, packet.data[1]);
        analogWrite(AF_LED_405, packet.data[2]);
        analogWrite(AF_LED_500, packet.data[3]);
        break;
    }
    case RC_AUGERBOARD_AUGERSERVO_DATA_ID: {
        // Soil Cache Servo & AF Lens Servo
        // int16_t* packet_data = (int16_t *)(&packet.data[0]);
        AFLensAngle = *((int16_t *)(&packet.data[0]));
        soilTrapdoorAngle = *((int16_t *)(&packet.data[2]));
        break;
    }
    case RC_AUGERBOARD_AUGERGIMBAL_DATA_ID: {
        // Gimbal Pan & Tilt Servos
        gimbalPanAngle = *((int16_t *)(&packet.data[0]));
        gimbalTiltAngle = *((int16_t *)(&packet.data[2]));
        break;
    }
    case RC_AUGERBOARD_POSITION_DATA_ID: {
        // wait for Angel to do his job
        // gets position data from smoco and sends it back to rovecomm

        break;
    }
    case RC_AUGERBOARD_AUGERSPEED_DATA_ID: {
        // wait for information about VESC from important people
        // gets speed data from VESC and sends it back to rovecomm

        break;
    }
    case RC_AUGERBOARD_LIMITSWITCH_DATA_ID: {

        // RoveComm.write(RC_AUGERBOARD_LIMITSWITCH_DATA_ID, RC_AUGERBOARD_LIMITSWITCH_DATA_COUNT, (data));
        break;
    }
    case RC_AUGERBOARD_ENVIRONMENTAL_DATA_ID: {

        break;
    }
    case RC_AUGERBOARD_AUGERCURRENT_DATA_ID: {
        // vesc thing

        break;
    }
    case RC_AUGERBOARD_SMOCOPING_DATA_ID: {
        // use the echoRequest function
        // we are cooked (wait for angel again)
        pingTime = (millis() - *(uint16_t *)(augerGantry.echoRequest(millis())));

        break;
    }
    }

    // Add testing features

    if (millis() - lastServoUpdate >= 10) {
        AFLens.write(AFLensAngle);
        soilTrapdoor.write(soilTrapdoorAngle);
        gimbalPan.write(gimbalPanAngle);
        gimbalTilt.write(gimbalTiltAngle);
        lastServoUpdate = millis();
        // Possibly needs roveComm input changed to 0 to 180
    }

    // add test button features
}

void estop() {
    watchdogStatus = 1;
    if (!watchdogOverride) {
        // disables gantry and auger motors
        // TO DO: Add commands to signal motors to stop
        augerGantry.stopAndReset()
    }
}

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
