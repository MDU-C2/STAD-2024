#include <Arduino.h>

#define PUL_PIN 2
#define DIR_PIN 4

#define PULSE_DELAY 3
#define LOOP_DELAY 300

#define CW 0
#define CCW 1

#define SET_CW() do { \
    digitalWrite(DIR_PIN, LOW); \
    delayMicroseconds(5);} while(0)

#define SET_CCW() do { \
    digitalWrite(DIR_PIN, HIGH); \
    delayMicroseconds(5);} while(0)

void setup() {
    pinMode(PUL_PIN, OUTPUT);                             
    pinMode(DIR_PIN, OUTPUT);
    Serial.begin(1000000);
}

void loop() {
    while (!Serial.available());
    reset:

    // PAYLOAD
    // x xxx xxxx xxxx xxxx
    // ^ ^^^^^^^^^^^^^^^^^^
    // |            |
    // direction    |
    //              |
    //         steps to move

    int first_byte = Serial.read();
    if (first_byte == -1) { goto skip; }

    int direction_bit = first_byte & 0b10000000;    
    if (direction_bit == CW) { SET_CW(); } else { SET_CCW(); }

    int msb_amount_byte = first_byte & 0b01111111;

    while (!Serial.available());

    int second_byte = Serial.read();
    if (second_byte == -1) { goto skip; }

    unsigned int amount_byte = (msb_amount_byte << 8) | second_byte;

    Serial.println(amount_byte, HEX);
    
    for (size_t i = 0; i < amount_byte; i++)
    {
        digitalWrite(PUL_PIN, HIGH);
        delayMicroseconds(PULSE_DELAY);
        digitalWrite(PUL_PIN, LOW);
        delayMicroseconds(PULSE_DELAY);

        delayMicroseconds(LOOP_DELAY);

        // hack : we don't want it to be stuck when another command is sent
        if (Serial.available()) {goto reset;}
    }

    skip:
    asm("nop");
}