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

    // Sensors
    pinMode(TEMP, INPUT);
    pinMode(MOISTURE, INPUT);

    // Autoflouresence
    pinMode(AF_LED_365, OUTPUT);
    pinMode(AF_LED_405, OUTPUT);
    pinMode(AF_LED_500, OUTPUT);
    pinMode(AF_WHITE_LED, OUTPUT);
    digitalWrite(AF_WHITE_LED, 0);
    analogWrite(AF_LED_365, 0);
    analogWrite(AF_LED_405, 0);
    analogWrite(AF_LED_500, 0);

    // Servos
    soilTrapdoor.attach(SOIL_TRAPDOOR_PWM, 500, 2500);
    AFLens.attach(AF_LENS_PWM, 544, 2400);
    gimbalPan.attach(SCI_GIMBAL_PAN, 600, 2400);
    gimbalTilt.attach(SCI_GIMBAL_TILT, 544, 2400);
    AFLens.write(180);
    soilTrapdoor.write(0);

    // Auger Motor
    auger_motor_can.begin(canSettings);
    vesc_can_init(send_msg, VESC_ID, 1);
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
    augerGantry.setRampRate(200);
}

RoveCommPacket packet;

void loop() {

    process_can_message();

    /*--------------------------RoveComm Updates--------------------------*/
    RoveComm.read(packet);

    switch (packet.dataId) {
    case RC_AUGERBOARD_AUGERAXIS_DATA_ID: {
        // sends a speed percentage to a smoco controlling the auger gantry
        augerAxisDecipercent = packet.i16data[0];

        if (augerAxisDecipercent != 0) {
            digitalWrite(GANTRY_LED, HIGH);
        } else {
            digitalWrite(GANTRY_LED, LOW);
        }
        augerGantry.driveOpenLoop(augerAxisDecipercent);
        feedWatchdog();

        break;
    }
    case RC_AUGERBOARD_LIMITSWITCHOVERRIDE_DATA_ID: {
        // sets auger axis motor object limit switch overide to be true for both FWD & REV
        uint8_t limitData = packet.i8data[0];
        bool forwardLimitOverride = limitData & 0x01;
        bool reverseLimitOverride = (limitData & 0x02) >> 1;
        augerGantry.configIgnoreLimits(forwardLimitOverride, reverseLimitOverride);

        break;
    }
    case RC_AUGERBOARD_CALIBRATEENCODER_DATA_ID: {
        // sends a command to smoco to drive gantry motor up until it triggers a limit switch, and sets that point to
        // zero for the encoder
        augerGantry.calibratePosition((INT16_MIN / 2), 0);
        break;
    }
    case RC_AUGERBOARD_AUGER_DATA_ID: {
        // sets the speed for the auger motor
        dutyCycle = packet.i16data[0] / 1000.0f;
        vesc_set_duty(VESC_ID, dutyCycle);

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
        watchdogOverride = packet.u8data[0];
        break;
    }
    case RC_AUGERBOARD_LED_DATA_ID: {
        // Sends PWM signal to AF LED for brightness & type
        digitalWrite(AF_WHITE_LED, packet.u8data[0] > 127 ? HIGH : LOW);
        analogWrite(AF_LED_365, packet.u8data[1]);
        analogWrite(AF_LED_405, packet.u8data[2]);
        analogWrite(AF_LED_500, packet.u8data[3]);
        break;
    }
        // Received data 0 goes to Autofluorescence and received data 1 goes to soil trapdoor.

    case RC_AUGERBOARD_AUGERSERVO_DATA_ID: {
        // Soil Cache Servo & AF Lens Servo
        if (AFLensAngle != packet.i16data[0]) {
            isAFLensMoving = true;
            lastAFLensUpdate = millis();
            AFLensAngle = packet.i16data[0];
        }
        soilTrapdoor.write(packet.i16data[1]);

        if (soilTrapdoorAngle != packet.i16data[1]) {
            isSoilTrapdoorMoving = true;
            lastSoilTrapdoorUpdate = millis();

            soilTrapdoorAngle = packet.i16data[1];
        }

        break;
    }
    case RC_AUGERBOARD_AUGERGIMBAL_DATA_ID: {
        // Gimbal Pan & Tilt Servos
        if (gimbalPanAngle != packet.i16data[0]) {
            isGimbalPanMoving = true;
            lastGimbalPanUpdate = millis();
        }
        if (gimbalTiltAngle != packet.i16data[1]) {
            isGimbalTiltMoving = true;
            lastGimbalTiltUpdate = millis();
        }
        gimbalPanAngle = packet.i16data[0];
        gimbalTiltAngle = packet.i16data[1];

        break;
    }
    }
    /*--------------------------SWITCHES AND TEST BUTTONS--------------------------*/

    // Direction Switch
    bool direction = digitalRead(DIR_SW);

    // Auger Gantry Button
    if (gantryButton.fallingEdge()) {
    } else if (!gantryButton.read()) {
        augerGantry.setSoftLimitPosition(INT32_MIN, INT32_MAX);
        digitalWrite(GANTRY_LED, HIGH);
        augerGantry.driveOpenLoop(direction ? INT16_MIN / 2 : INT16_MAX / 2);
        feedWatchdog();
    }
    if (gantryButton.risingEdge()) {
        digitalWrite(GANTRY_LED, LOW);
        augerGantry.driveOpenLoop(0);
    }

    // Auger Motor Button
    if (augerButton.fallingEdge()) {
    } else if (!augerButton.read()) {
        digitalWrite(VESC_LED, HIGH);
        vesc_set_duty(VESC_ID, direction ? -1.0f : 1.0f);
        feedWatchdog();
    }
    if (augerButton.risingEdge()) {
        digitalWrite(VESC_LED, LOW);
        vesc_set_duty(VESC_ID, 0.0f);
    }

    // Servo Updates (will try to set position every 10ms if position is changing)
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

        AFLens.write(AFLensAngle);
        soilTrapdoor.write(soilTrapdoorAngle);
        gimbalPan.write(gimbalPanAngle);
        gimbalTilt.write(gimbalTiltAngle);
        lastServoUpdate = millis();
    }

    // Servo Movement LEDs (stay on for LED_DURATION ms after position changes)
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
    // Temperature and Humidity Sensors
    // TO DO: Will need to be calibrated
    /*--------------------------Sensor Readings--------------------------*/
    if (millis() - lastTempRead > 100) {
        int tempReading = analogRead(TEMP);
        if ((tempReading < veryCold) || (tempReading > veryWarm)) {
            tempReading = std::max(std::min(tempReading, veryWarm), veryCold);
        }
        tempCelcius = calibratedAnalogMapTemp(tempReading);
        if (!(dataCountTemp > 9)) {
            temperatureSum += tempCelcius;
            dataCountTemp++;
        } else {
            avgTemp = temperatureSum / 10.0f;
            dataCountTemp = 0;
            temperatureSum = 0.0f;
        }
        lastTempRead = millis();
    }

    if (millis() - lastHumidityRead > 100) {
        uint16_t humidityReading = analogRead(MOISTURE);
        humidity = calibratedAnalogMapHumidity(humidityReading);
        lastHumidityRead = millis();
        if (!(dataCountHumidity > 9)) {
            humiditySum += humidity;
            dataCountHumidity++;
        } else {
            avgHumidity = humiditySum / 10.0f - 70.0f;
            dataCountHumidity = 0;
            humiditySum = 0.0f;
        }
        lastHumidityRead = millis();
    }

    // Test Buttons Update (required when using bounce library)
    gantryButton.update();
    augerButton.update();
    /*if (!(digitalRead(SPARE_SW_12V))) { // TO DO: Finish & Remove
        augerGantry.calibratePosition((INT16_MAX / 2), 0);
    }
    Serial.println(augerGantry.getPosition()); */
}

void estop() {
    if (!watchdogOverride) {
        // disables gantry and auger motors
        vesc_set_duty(VESC_ID, 0.0f);
        augerGantry.driveOpenLoop(0);
        digitalWrite(GANTRY_LED, LOW);
        digitalWrite(VESC_LED, LOW);
        // Serial.println("E-STOP ACTIVATED");
    }
}

void feedWatchdog() { Watchdog.begin(estop, WATCHDOG_TIMEOUT); }

// Telemetry function that runs every TELEMETRY_INTERVAL ms, sends data to the RoveComm
void telemetry() {

    // Retrieves and processes CAN messages from the auger gantry and auger motor
    process_can_message();

    // Auger Gantry Position
    float gantryPosition = augerGantry.getPosition() * INCHES_PER_STEP; // TO DO: Add inches multiplier
    RoveComm.write(RC_AUGERBOARD_POSITION_DATA_ID, RC_AUGERBOARD_POSITION_DATA_COUNT, &gantryPosition);

    // Auger speed
    vesc_get_values(VESC_ID);
    RoveComm.write(RC_AUGERBOARD_AUGERSPEED_DATA_ID, RC_AUGERBOARD_AUGERSPEED_DATA_COUNT, &augerSpeed);

    // Limit Switch Data
    uint8_t limitSwitchValues = (augerGantry.getLimitSwitchA()) | (augerGantry.getLimitSwitchB() ? (1 << 1) : 0);
    RoveComm.write(RC_AUGERBOARD_LIMITSWITCH_DATA_ID, RC_AUGERBOARD_LIMITSWITCH_DATA_COUNT, &limitSwitchValues);

    // Sensor Data
    float environmentalData[2] = {avgTemp, avgHumidity};
    RoveComm.write(RC_AUGERBOARD_ENVIRONMENTAL_DATA_ID, RC_AUGERBOARD_ENVIRONMENTAL_DATA_COUNT, environmentalData);

    // Auger Current
    RoveComm.write(RC_AUGERBOARD_AUGERCURRENT_DATA_ID, RC_AUGERBOARD_AUGERCURRENT_DATA_COUNT, &augerCurrent);

    // Auger Gantry Ping Time
    augerGantry.ping();
    uint16_t pingTime = augerGantry.getPingTime();
    RoveComm.write(RC_AUGERBOARD_SMOCOPING_DATA_ID, RC_AUGERBOARD_SMOCOPING_DATA_COUNT, &pingTime);
}

float analogMap(uint16_t measurement, uint16_t fromADC, uint16_t toADC, float fromAnalog, float toAnalog) {
    float slope = (toAnalog - fromAnalog) / (toADC - fromADC);
    float b = fromAnalog + (slope * (-fromADC));
    return b + (measurement * slope);
}

float calibratedAnalogMapHumidity(int measurement) { return analogMap(measurement, veryWet, veryDry, 0.0f, 100.0f); }

// TO DO: Add calibration values
float calibratedAnalogMapTemp(int measurement) {
    if (measurement < middleCold) {
        return analogMap(measurement, veryCold, middleCold, 1.6f, 33.7f);
    } else {
        return analogMap(measurement, middleCold, veryWarm, 33.7f, 98.8f);
    }
}

bool send_msg(uint32_t id, uint8_t *data, uint8_t len) {
    CANMessage msg;
    msg.id = id;
    msg.ext = true;
    msg.len = len;
    memcpy(msg.data, data, len);
    int result = ACAN_T4::can2.tryToSendReturnStatus(msg);
    Serial.println(result);
    return result == 0;
}

void response_callback(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    if (command == COMM_GET_VALUES) {
        vesc_values_t status;
        if (vesc_parse_get_values(data, len, &status)) {
            augerSpeed = status.rpm / 24;
            augerCurrent = status.current_motor;
            // Serial.println("e");
        } else {
            Serial.println("Failed to parse status message 1");
        }
    }
}

void process_can_message() {
    while (auger_axis_can.available()) {
        CANMessage msg;
        auger_axis_can.receive(msg);
        // Serial.printf("Received CAN packet with ID %d\n", msg.id);
        // Checks if message is from auger gantry (extended) or from auger motor, and processes accordingly
        if (msg.ext) {
            vesc_process_can_frame(msg.id, msg.data, msg.len);
            // Serial.println("Sending packet to VESC");
        } else {
            // Serial.printf("Sending packet to Smoco (%d)\n", msg.id & 0xF);
            if ((msg.id & 0xF) == 13) {
                Serial.printf("ERROR:::%d:::\n", ((SmocoCANMessage *)msg.data)->commandError.commandID);
            }
            augerGantry.sync(msg);
        }
    }
}
