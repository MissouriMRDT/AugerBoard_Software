#include "AugerBoard.h"


void setup() {
    Serial.begin(115200);
    AUGER_SERIAL.begin(115200);
    // while(!AUGER_SERIAL);

    // Test Buttons
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);

    // Direction Switch
    pinMode(DIR_SW, INPUT);

    // Sensors
    pinMode(TEMP, INPUT);
    pinMode(HUMIDITY, INPUT);

    // Autofluorescence
    pinMode(UVLED, OUTPUT);

    AugerAxisMotor.init();
    SpareMotor.init();

    AugerAxis.attachEncoder(&AugerAxisEncoder);
    
    pinMode(LIMITSWITCH1, INPUT_PULLDOWN);
    pinMode(LIMITSWITCH2, INPUT_PULLDOWN);
    pinMode(LIMITSWITCH3, INPUT_PULLDOWN);
    AugerAxisFWDLimit.configInvert(true);
    AugerAxisRVSLimit.configInvert(false);
    AugerAxis.attachHardLimits(&AugerAxisRVSLimit, &AugerAxisFWDLimit);

    AugerAxis.Motor()->configRampRate(5000);
    AugerMotor.configRampRate(5000);
    SpareMotor.configRampRate(5000);

    AugerAxisEncoder.begin([] { AugerAxisEncoder.handleInterrupt(); });

    AugerAxis.Motor()->configMaxOutputs(-1000, 1000);
    AugerAxis.Motor()->configMinOutputs(0, 0);

    multiplexer.attach(SERVO_PWM1);

    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_AUGERBOARD_IPADDRESS);
    Serial.println("Complete");

    Telemetry.begin(telemetry, TELEMETRY_INTERVAL);

    //servoStartup
}

void loop() {

    RoveCommPacket packet;
    RoveComm.read(packet);


    switch (packet.dataId) {
    case RC_AUGERBOARD_AUGERAXIS_OPENLOOP_DATA_ID: {
        augerAxisDecipercent = *((int16_t *)packet.data);
        feedWatchdog();

        break;
    }
    case RC_AUGERBOARD_LIMITSWITCHOVERRIDE_DATA_ID: {
        uint8_t data = ((uint8_t *)packet.data)[0];

        AugerAxis.overrideForwardHardLimit(data & (1 << 0));
        AugerAxis.overrideReverseHardLimit(data & (1 << 1));

        break;
    }
    case RC_AUGERBOARD_UVLED_DATA_ID: {
        enableUVLED(packet.data[0]);
        break;
    }
    case RC_AUGERBOARD_REQUESTHUMIDITY_DATA_ID: {
        RoveComm.write(RC_AUGERBOARD_HUMIDITY_DATA_ID, readHumidity());
        break;
    }
    case RC_AUGERBOARD_REQUESTTEMPERATURE_DATA_ID: {
        RoveComm.write(RC_AUGERBOARD_TEMPERATURE_DATA_ID, readTemperature());
        break;
    }
    case RC_AUGERBOARD_AUGERMULTIPLEXERSERVO_DATA_ID: {
        multiplexerAngle = *((int16_t *)packet.data);
        feedWatchdog();

        break;
    }
    case RC_AUGERBOARD_AUGER_DATA_ID: { // CHANGE TO VESC
        augerDecipercent = *((int16_t *)packet.data);
        feedWatchdog();

        break;
    }
    case RC_AUGERBOARD_WATCHDOGOVERRIDE_DATA_ID: { 
        watchdogOverride = *((uint8_t *)packet.data);

        break;
    }
    }

    bool direction = digitalRead(DIR_SW);

    // AugerAxis 
    if (!digitalRead(SW1)) {
        AugerMotor.drive((direction ? -900 : 900));
    } else {
        AugerMotor.drive(augerDecipercent);
    }
    // Auger
    if (!digitalRead(SW2)) {
        AugerAxis.drive((direction ? -900 : 900));
    } else {
        AugerAxis.drive(augerAxisDecipercent);
    }

    // Sensors
    temperature = readTemperature();
    humidity = readHumidity();

    // Multiplexer
    // 0   Degrees is soil cache
    // 127 Degress is AF
    if (millis() - lastMultiplexerUpdate >= 10)
    {
        // Multiplxer Gimbal
        if (!digitalRead(SW3)) {
            multiplexerAngle = direction ? multiplexerAngle - 1 : multiplexerAngle + 1;

            if (multiplexerAngle < SOIL_CACHE_ANGLE)
                multiplexerAngle = SOIL_CACHE_ANGLE;
            else if (multiplexerAngle > AF_ANGLE)
                multiplexerAngle = AF_ANGLE;
        }

        multiplexer.write(multiplexerAngle);
        Serial.printf("Multiplexer %d\n", multiplexerAngle);
        lastMultiplexerUpdate = millis();
    }
}

float readTemperature() {
    // TODO: implement when thermocouple is installed
    uint16_t tempReading = analogRead(TEMP);
    //Temperature = (Vout - 1.25) / 0.005 V. So for example, if the voltage is 1.5VDC, the temperature is (1.5 - 1.25) / 0.005 = 50°C
    //float tempCelcius = ((((tempReading/1023.0)*3.3) - 1.25)/0.005);
    tempCelcius = analogMap(tempReading, 388, 1023, 0.0, 410.0);
    float tempFarenheit = (((tempCelcius)*(9/5)) + 32); //(0°C × 9/5) + 32 = 32°F
    return tempCelcius;
}

float readHumidity() {
    int humidityData = analogRead(HUMIDITY);
    /*float humidityPercent =
        ((100.0 / humidityRange) * humidityData) +
        (100 -
            (veryWet *
            (100.0 / humidityRange))); // (100/range) is the slope. The y-intercept is calculated by taking the slope
                                        // times the value that's 100% Humidity and adding that to the y level 100.
    */
    float humidityPercent = analogMap(humidityData, veryDry, veryWet, 0.0f, 100.0f);
    return humidityPercent;
}

void telemetry() {

    // Temperature
    RoveComm.write(RC_AUGERBOARD_TEMPERATURE_DATA_ID, temperature);

    // Humidity
    RoveComm.write(RC_AUGERBOARD_HUMIDITY_DATA_ID, humidity);

    // Encoder position
    float position = AugerAxisEncoder.readDegrees();
    RoveComm.write(RC_AUGERBOARD_POSITION_DATA_ID, position);

    // Limit
    uint8_t limitSwitchValues = (AugerAxis.atForwardHardLimit()) | (AugerAxis.atReverseHardLimit());
    RoveComm.write(RC_AUGERBOARD_LIMITSWITCHTRIGGERED_DATA_ID, limitSwitchValues);

    // Watchdog status
    RoveComm.write(RC_AUGERBOARD_WATCHDOGSTATUS_DATA_ID, watchdogStatus);
}

float analogMap(uint16_t measurement, uint16_t fromADC, uint16_t toADC, float fromAnalog, float toAnalog) {
    float slope = (toAnalog - fromAnalog) / (toADC - fromADC);
    float b = fromAnalog + (slope * (-fromADC));
    return b + (measurement * slope);
}

void enableUVLED(bool enable) {
    if (enable && cooled && !digitalRead(UVLED)) {
        analogWrite(UVLED, MAX_UVLED_LEVEL);
        cooled = false;
        UVLEDWatchdog.begin(estopUVLED, MAX_UVLED_ON_PERIOD);
    }
}

void estopUVLED() {
    analogWrite(UVLED, 0);
    cooled = false;
    UVLEDWatchdog.end();
    UVLEDCooldownWatchdog.begin(cooldownUVLED, UVLED_COOLING_PERIOD);
}

void cooldownUVLED()
{
    cooled = true;
    UVLEDCooldownWatchdog.end();
}

void estop() {
    watchdogStatus = 1;
    if (!watchdogOverride) {
        AugerAxis.drive(0);
        AugerMotor.drive(0);

        augerAxisDecipercent = 0;
        augerDecipercent = 0;
    }
}

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
