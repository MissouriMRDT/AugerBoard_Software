#include "AugerBoard.h"

    int pp = 1000;
    int pp2 = 0;
void setup() {
    Serial.begin(115200);

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
    AugerMotor.init();

    AugerAxis.attachEncoder(&AugerAxisEncoder);
    

    pinMode(LIMITSWITCH1, INPUT_PULLDOWN);
    pinMode(LIMITSWITCH2, INPUT_PULLDOWN);
    pinMode(LIMITSWITCH3, INPUT_PULLDOWN);
    pinMode(LIMITSWITCH4, INPUT_PULLDOWN);
    AugerAxisFWDLimit.configInvert(true);
    AugerAxisRVSLimit.configInvert(true);
    AugerAxis.attachHardLimits(&AugerAxisRVSLimit, &AugerAxisFWDLimit);

    AugerAxis.Motor()->configRampRate(5000);
    AugerMotor.configRampRate(5000);
    SpareMotor.configRampRate(5000);


    AugerAxisEncoder.begin([] { AugerAxisEncoder.handleInterrupt(); });
    AugerEncoder.begin([] { AugerEncoder.handleInterrupt(); });

    AugerAxis.Motor()->configMaxOutputs(-1000, 1000);
    AugerAxis.Motor()->configMinOutputs(0, 0);

    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_AUGERBOARD_IPADDRESS);
    Serial.println("Complete");

    Telemetry.begin(telemetry, TELEMETRY_INTERVAL);
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
    case RC_AUGERBOARD_AUGER_DATA_ID: {
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
    if (!digitalRead(SW2)) {
        AugerAxis.drive((direction ? -900 : 900));
    } else {
        AugerAxis.drive(augerAxisDecipercent);
    }
    // Auger
    if (!digitalRead(SW1)) {
        AugerMotor.drive((direction ? -900 : 900));
    } else {
        AugerMotor.drive(augerDecipercent);
    }
    // Spare Motor
    if (!digitalRead(SW3)) {
        SpareMotor.drive((direction ? -900 : 900));
    } else {
        SpareMotor.drive(0);
    }

    // Sensors
    temperature = readTemperature();
    humidity = readHumidity();
}

float readTemperature() {
    // TODO: implement when thermocouple is installed
    // uint16_t reading = analogRead(TEMP);
    // return analogMap(reading, something)
    float fluctuation = ((random() % 200) - 100)/ 100.0;
    return 5.0 + fluctuation;
}

float readHumidity() {
    int humidityData = analogRead(HUMIDITY);
    float humidityPercent =
        ((100.0 / humidityRange) * humidityData) +
        (100 -
            (veryWet *
            (100.0 / humidityRange))); // (100/range) is the slope. The y-intercept is calculated by taking the slope
                                        // times the value that's 100% Humidity and adding that to the y level 100.

    
    return 38.0 + ((random() % 200) - 100) / 100.0;
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
    // TODO: implement this
    return 6.9f;
}

void enableUVLED(bool enable) {
    if (enable) {
        analogWrite(UVLED, MAX_UVLED_LEVEL);
        UVLEDWatchdog.begin(estopUVLED, MAX_UVLED_ON_PERIOD);
    } else {
        analogWrite(UVLED, 0);
    }
}

void estopUVLED() {
    analogWrite(UVLED, 0);
    UVLEDWatchdog.end();
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
