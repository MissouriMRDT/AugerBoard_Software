#include "AugerBoard.h"

void setup() {
    // put your setup code here, to run once:

    Serial.begin(115200);

    // Test Buttons
    pinMode(SW1,INPUT_PULLUP);
    pinMode(SW2,INPUT_PULLUP);
    pinMode(SW3,INPUT_PULLUP);

    // Direction Switch
    pinMode(DIR_SW, INPUT); 
}

void loop() {
    // put your main code here, to run repeatedly:
}
