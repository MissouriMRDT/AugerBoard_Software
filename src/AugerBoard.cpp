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

    // Indicator LEDs
    pinMode(GIMBAL_TILT_LED, OUTPUT);
    pinMode(SOIL_TD_LED, OUTPUT);
    pinMode(VESC_LED, OUTPUT);
    pinMode(AFF_LED, OUTPUT);
    pinMode(GIMBAL_PAN_LED, OUTPUT);
    pinMode(GANTRY_LED, OUTPUT);
    digitalWrite(GIMBAL_TILT_LED, LOW);
    digitalWrite(SOIL_TD_LED, LOW);
    digitalWrite(VESC_LED, LOW);
    digitalWrite(AFF_LED, LOW);
    digitalWrite(GIMBAL_PAN_LED, LOW);
    digitalWrite(GANTRY_LED, LOW);
    // LEDs only run when moving servos' positions or recieveing new position data for aprox. 500ms

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
    soilTrapdoor.attach(SOIL_TRAPDOOR_PWM, 544, 2400);
    AFLens.attach(AF_LENS_PWM, 544, 2400);
    gimbalPan.attach(SCI_GIMBAL_PAN, 600, 2400);
    gimbalTilt.attach(SCI_GIMBAL_TILT, 544, 2400);

    // Auger Motor
    auger_motor_can.begin(canSettings);
    vesc_can_init(send_msg, 53, 1);
    vesc_set_response_callback(response_callback);
    pinMode(STBY_1, OUTPUT);
    pinMode(STBY_2, OUTPUT);
    digitalWrite(STBY_1, LOW);
    digitalWrite(STBY_2, LOW);

    // RoveComm Initialization
    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_AUGERBOARD_IPADDRESS);
    Serial.println("Complete");

    // Telemetry Timer
    Telemetry.begin(telemetry, TELEMETRY_INTERVAL);

    delay(1000);
    augerGantry.setSoftLimitPosition(INT32_MIN, INT32_MAX);
    // augerGantry.setPID(1, 0, 0);
    augerGantry.setLowPassSmoothingFactor(UINT16_MAX);
}

RoveCommPacket packet;

void loop() {
    process_can_message();

    RoveComm.read(packet);

    switch (packet.dataId) {
    case RC_AUGERBOARD_AUGERAXIS_DATA_ID: {
        // sends a speed percentage to a smoco controlling the auger gantry
        augerAxisDecipercent = *(int16_t *)packet.data;

        if (augerAxisDecipercent != 0) {
            digitalWrite(GANTRY_LED, HIGH);
        } else {
            digitalWrite(GANTRY_LED, LOW);
        }
        augerGantry.openLoopDrive(augerAxisDecipercent, augerGantry.getignoreLimitVariable());

        feedWatchdog();

        break;
    }
    case RC_AUGERBOARD_LIMITSWITCHOVERRIDE_DATA_ID: {
        // sets auger axis motor object limit switch overide to be true for both FWD & REV
        uint8_t data = *(uint8_t *)packet.data;
        augerGantry.setIgnoreLimitVariable((data & (1 << 0)) || (data & (1 << 1)));
        Serial.println(augerGantry.getignoreLimitVariable());

        break;
    }
    case RC_AUGERBOARD_CALIBRATEENCODER_DATA_ID: {
        // sends a command to smoco to drive gantry motor up until it triggers a limit switch, and sets that point to
        // zero for the encoder
        augerGantry.startPositionCalibration((INT16_MIN / 4), 0);
        Serial.println("Calibrating Auger Gantry Encoder");
        break;
    }
    case RC_AUGERBOARD_AUGER_DATA_ID: {
        // sets the speed for the auger motor
        dutyCycle = ((*(int16_t *)packet.data) / 1000.0f);
        vesc_set_duty(53, dutyCycle);

        if (dutyCycle != 0) {
            digitalWrite(VESC_LED, HIGH);
        } else {
            digitalWrite(VESC_LED, LOW);
        }
        feedWatchdog();

        break;
    }
    case RC_AUGERBOARD_WATCHDOGOVERRIDE_DATA_ID: {
        // disables the watchdog interrupt
        watchdogOverride = *((uint8_t *)packet.data);
        Serial.println(watchdogOverride);
        break;
    }
    case RC_AUGERBOARD_LED_DATA_ID: {
        // Sends PWM signal to AF LED for brightness & type
        analogWrite(AF_WHITE_LED, packet.data[0]);
        analogWrite(AF_LED_365, packet.data[1]);
        analogWrite(AF_LED_405, packet.data[2]);
        analogWrite(AF_LED_500, packet.data[3]);
        Serial.println(packet.data[0]);
        Serial.println(packet.data[1]);
        Serial.println(packet.data[2]);
        Serial.println(packet.data[3]);
        break;
    }
    case RC_AUGERBOARD_AUGERSERVO_DATA_ID: {
        // Soil Cache Servo & AF Lens Servo
        if (AFLensAngle != *((int16_t *)(&packet.data[0]))) {
            isAFLensMoving = true;
            lastAFLensUpdate = millis();
            if (soilTrapdoorAngle != *((int16_t *)(&packet.data[2]))) {
                isSoilTrapdoorMoving = true;
                lastSoilTrapdoorUpdate = millis();
            }
            AFLensAngle = *((int16_t *)(&packet.data[0]));
            soilTrapdoorAngle = *((int16_t *)(&packet.data[2]));
            Serial.println("Lens Angle Updated");
            Serial.println(AFLensAngle);
            Serial.println("Soil Trapdoor Angle Updated");
            Serial.println(soilTrapdoorAngle);
            break;
        }
    }
    case RC_AUGERBOARD_AUGERGIMBAL_DATA_ID: {
        // Gimbal Pan & Tilt Servos
        if (gimbalPanAngle != *((int16_t *)(&packet.data[0]))) {
            isGimbalPanMoving = true;
            lastGimbalPanUpdate = millis();
        }
        if (gimbalTiltAngle != *((int16_t *)(&packet.data[2]))) {
            isGimbalTiltMoving = true;
            lastGimbalTiltUpdate = millis();
        }
        gimbalPanAngle = *((int16_t *)(&packet.data[0]));
        gimbalTiltAngle = *((int16_t *)(&packet.data[2]));
        Serial.println(gimbalPanAngle);
        Serial.println(gimbalTiltAngle);
        break;
    }
    }

    // Direction Switch
    bool direction = digitalRead(DIR_SW);

    // Auger Gantry Button
    if (gantryButton.fallingEdge()) {
    } else if (!gantryButton.read()) {
        augerGantry.setSoftLimitPosition(INT32_MIN, INT32_MAX);
        // augerGantry.setPID(1, 0, 0);
        augerGantry.setLowPassSmoothingFactor(UINT16_MAX);
        digitalWrite(GANTRY_LED, HIGH);
        augerGantry.openLoopDrive(direction ? INT16_MIN / 4 : INT16_MAX / 4);
        feedWatchdog();
    }
    if (gantryButton.risingEdge()) {
        digitalWrite(GANTRY_LED, LOW);
        augerGantry.openLoopDrive(0, augerGantry.getignoreLimitVariable());
    }

    // Auger Motor Button

    if (augerButton.fallingEdge()) {
    } else if (!augerButton.read()) {
        digitalWrite(VESC_LED, HIGH);
        vesc_set_duty(53, direction ? -0.1f : 0.1f);
        feedWatchdog();
    }
    if (augerButton.risingEdge()) {
        digitalWrite(VESC_LED, LOW);
        vesc_set_duty(53, 0.0f);
    }

    // Servo Updates
    if (millis() - lastServoUpdate >= 10) {

        if (!digitalRead(AF_LENS_SW)) {
            AFLensAngle = direction ? AFLensAngle - 1 : AFLensAngle + 1;
            isAFLensMoving = true;
            lastAFLensUpdate = millis();
        }
        if (!digitalRead(SOIL_DOOR_SW)) {
            soilTrapdoorAngle = direction ? soilTrapdoorAngle - 1 : soilTrapdoorAngle + 1;
            isSoilTrapdoorMoving = true;
            lastSoilTrapdoorUpdate = millis();
        }
        if (!digitalRead(GIMBAL_PAN_SW)) {
            gimbalPanAngle = direction ? gimbalPanAngle - 1 : gimbalPanAngle + 1;
            isGimbalPanMoving = true;
            lastGimbalPanUpdate = millis();
        }
        if (!digitalRead(GIMBAL_TILT_SW)) {
            gimbalTiltAngle = direction ? gimbalTiltAngle - 1 : gimbalTiltAngle + 1;
            isGimbalTiltMoving = true;
            lastGimbalTiltUpdate = millis();
        }
        /* TO DO: Update servo positions
       if (AFLensAngle < AF_LENS_ANGLE_1) {
           AFLensAngle = AF_LENS_ANGLE_1;
       } else if (AFLensAngle < AF_LENS_ANGLE_2) {
           AFLensAngle = AF_LENS_ANGLE_2;
       } else if (AFLensAngle < AF_LENS_ANGLE_3) {
           AFLensAngle = AF_LENS_ANGLE_3;
       } else if (AFLensAngle < AF_LENS_ANGLE_BLANK) {
           AFLensAngle = AF_LENS_ANGLE_BLANK;
       } */
        // TO DO: Update cache positions
        if (soilTrapdoorAngle < SOIL_CACHE_ANGLE_LEFT) {
            soilTrapdoorAngle = SOIL_CACHE_ANGLE_LEFT;
        } else if (soilTrapdoorAngle > SOIL_CACHE_ANGLE_RIGHT) {
            soilTrapdoorAngle = SOIL_CACHE_ANGLE_RIGHT;
        }
        AFLens.write(AFLensAngle);
        soilTrapdoor.write(soilTrapdoorAngle);
        gimbalPan.write(gimbalPanAngle);
        gimbalTilt.write(gimbalTiltAngle);
        lastServoUpdate = millis();
    }
    if (isAFLensMoving) {
        digitalWrite(AFF_LED, HIGH);
        if (millis() - lastAFLensUpdate > LED_DURATION) {
            isAFLensMoving = false;
            digitalWrite(AFF_LED, LOW);
        }
    }
    if (isSoilTrapdoorMoving) {
        digitalWrite(SOIL_TD_LED, HIGH);
        if (millis() - lastSoilTrapdoorUpdate > LED_DURATION) {
            isSoilTrapdoorMoving = false;
            digitalWrite(SOIL_TD_LED, LOW);
        }
    }
    if (isGimbalPanMoving) {
        digitalWrite(GIMBAL_PAN_LED, HIGH);
        if (millis() - lastGimbalPanUpdate > LED_DURATION) {
            isGimbalPanMoving = false;
            digitalWrite(GIMBAL_PAN_LED, LOW);
        }
    }
    if (isGimbalTiltMoving) {
        digitalWrite(GIMBAL_TILT_LED, HIGH);
        if (millis() - lastGimbalTiltUpdate > LED_DURATION) {
            isGimbalTiltMoving = false;
            digitalWrite(GIMBAL_TILT_LED, LOW);
        }
    }
    // Possibly needs roveComm input changed to 0 to 180
    // Temperature and Humidity Sensor  s
    // TO DO: Will need to be calibrated
    if (millis() - lastTempRead > 100) {
        uint16_t tempReading = analogRead(TEMP);
        tempCelcius = calibratedAnalogMapTemp(tempReading);
        if (!(dataCountTemp >= 10)) {
            temperature += tempCelcius;
            dataCountTemp++;
        } else {
            avgTemp = temperature / 10.0f;

            dataCountTemp = 0;
        }
        lastTempRead = millis();
    }
    if (millis() - lastHumidityRead > 100) {
        uint16_t humidityReading = analogRead(MOISTURE);
        humidity = calibratedAnalogMapHumidity(humidityReading);
        lastHumidityRead = millis();
        if (!(dataCountHumidity >= 10)) {
            humidity += humidity;
            dataCountHumidity++;
        } else {
            avgHumidity = humidity / 10.0f;

            dataCountHumidity = 0;
        }
    }
    gantryButton.update();
    augerButton.update();
}
// TO DO: redo e-stop commands for buttons
void estop() {
    watchdogStatus = 1;
    if (!watchdogOverride) {
        // disables gantry and auger motors
        vesc_set_duty(53, 0.0f);
        augerGantry.openLoopDrive(0, augerGantry.getignoreLimitVariable());
        // augerGantry.stopAndReset();
        digitalWrite(GANTRY_LED, LOW);
        digitalWrite(VESC_LED, LOW);
        Serial.println("E-STOP ACTIVATED");
    }
}

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
// TO DO: figure out VESC telemetry
void telemetry() {
    process_can_message();
    // Auger Gantry Position
    float gantryPosition = augerGantry.getAngleVariable();
    // float gantryPosition = augerGantry.m_position;
    RoveComm.write(RC_AUGERBOARD_POSITION_DATA_ID, RC_AUGERBOARD_POSITION_DATA_COUNT, &gantryPosition);
    // Auger speed
    vesc_get_values(53);

    RoveComm.write(RC_AUGERBOARD_AUGERSPEED_DATA_ID, RC_AUGERBOARD_AUGERSPEED_DATA_COUNT, &augerSpeed);
    // Limit Switch Data
    uint8_t limitSwitchValues =
        (augerGantry.getLimitSwitchAVariable()) | (augerGantry.getLimitSwitchBVariable() ? (1 << 1) : 0);
    // uint8_t limitSwitchValues = (augerGantry.m_limitSwitchA) | (augerGantry.m_limitSwitchB ? (1 << 1) : 0);
    RoveComm.write(RC_AUGERBOARD_LIMITSWITCH_DATA_ID, RC_AUGERBOARD_LIMITSWITCH_DATA_COUNT, &limitSwitchValues);
    // Sensor Data
    float environmentalData[6] = {avgTemp, avgHumidity, 0.0f, 0.0f, 0.0f, 0.0f};
    RoveComm.write(RC_AUGERBOARD_ENVIRONMENTAL_DATA_ID, RC_AUGERBOARD_ENVIRONMENTAL_DATA_COUNT, environmentalData);
    // Auger Current
    RoveComm.write(RC_AUGERBOARD_AUGERCURRENT_DATA_ID, RC_AUGERBOARD_AUGERCURRENT_DATA_COUNT, &augerCurrent);
    // Auger Gantry Ping Time
    // augerGantry.ping();
    augerGantry.smocoPing();

    // uint16_t pingTime = augerGantry.m_pingTime;
    uint16_t pingTime = augerGantry.getPingTimeVariable();
    RoveComm.write(RC_AUGERBOARD_SMOCOPING_DATA_ID, RC_AUGERBOARD_SMOCOPING_DATA_COUNT, &pingTime);
}

float analogMap(uint16_t measurement, uint16_t fromADC, uint16_t toADC, float fromAnalog, float toAnalog) {
    float slope = (toAnalog - fromAnalog) / (toADC - fromADC);
    float b = fromAnalog + (slope * (-fromADC));
    return b + (measurement * slope);
}
// TO DO: Add calibration values
float calibratedAnalogMapHumidity(int measurement) {
    if (measurement < middleWet) {
        return analogMap(measurement, veryWet, middleWet, 0.0f, 50.0f);
    } else {
        return analogMap(measurement, middleWet, veryDry, 50.0f, 100.0f);
    }
}
// TO DO: Add calibration values
float calibratedAnalogMapTemp(int measurement) {
    if (measurement < middleCold) {
        return analogMap(measurement, veryCold, middleCold, 0.0f, 50.0f);
    } else {
        return analogMap(measurement, middleCold, veryWarm, 50.0f, 100.0f);
    }
}

bool send_msg(uint32_t id, uint8_t *data, uint8_t len) {
    CANMessage msg;
    msg.id = id;
    msg.ext = true;
    msg.len = len;
    memcpy(msg.data, data, len);
    int result = ACAN_T4::can2.tryToSendReturnStatus(msg);
    // Serial.println(result);
    return result == 0;
}

void response_callback(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    if (command == COMM_GET_VALUES) {
        vesc_values_t status;
        if (vesc_parse_get_values(data, len, &status)) {
            augerSpeed = status.rpm / 24;
            augerCurrent = status.current_motor;
        } else {
            Serial.println("Failed to parse status message 1");
        }
    }
}

// TO DO: add ping time function for VESC
// TO DO: after debugging remove prints and tidy wording
void process_can_message() {
    while (auger_axis_can.available()) {
        CANMessage msg;
        auger_axis_can.receive(msg);
        Serial.printf("Received CAN packet with ID %d\n", msg.id);
        if (msg.ext) {
            Serial.println("Sending packet to VESC");
            vesc_process_can_frame(msg.id, msg.data, msg.len);
        } else {
            Serial.printf("Sending packet to Smoco (%d)\n", msg.id & 0xF);
            /* if ((msg.id & 0xF) == 13) {
                Serial.printf("ERROR:::%d:::\n", ((SmocoCANMessage *)msg.data)->commandError.commandID);
            }
            augerGantry.sync(msg); */
        }
    }
}
