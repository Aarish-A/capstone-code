#include "motors.h"

// Global variables
int dutyCycle = 0;  // Duty Cycle to pass to motor

void initMotor() {
    pinMode(PIN_MOTOR_FORWARD_A, OUTPUT);
    pinMode(PIN_MOTOR_FORWARD_B, OUTPUT);
    pinMode(PIN_MOTOR_REVERSE_A, OUTPUT);
    pinMode(PIN_MOTOR_REVERSE_B, OUTPUT);

    ledcSetup(0, 1000, 8);
    ledcSetup(1, 1000, 8);

    ledcAttachPin(PIN_MOTOR_FORWARD_B, 0);
    ledcAttachPin(PIN_MOTOR_REVERSE_B, 1);
}

void setMotor(int dutyCycle) {
    bool reverse = dutyCycle > 0 ? true : false;

    // TODO: Time execution for this part of the code
    ledcDetachPin(PIN_MOTOR_FORWARD_B);
    ledcDetachPin(PIN_MOTOR_REVERSE_B);

    digitalWrite(PIN_MOTOR_FORWARD_A, LOW);
    digitalWrite(PIN_MOTOR_FORWARD_B, LOW);
    digitalWrite(PIN_MOTOR_REVERSE_A, LOW);
    digitalWrite(PIN_MOTOR_REVERSE_B, LOW);
    // ledcWrite(0, 0);  // Channel 0 = PIN_MOTOR_FORWARD_B
    // ledcWrite(1, 0);  // Channel 1 = PIN_MOTOR_REVERSE_B

    delay(1);  // Wait for MOSFETS to reset, can be much lower, 1ms for now

    if (!reverse) {
        if (sensors.helix < HELIX_MAX_ANGLE) {
            ledcAttachPin(PIN_MOTOR_FORWARD_B, 0);
            digitalWrite(PIN_MOTOR_FORWARD_A, HIGH);
            ledcWrite(0, abs(dutyCycle));
        }
    } else if (reverse) {
        if (sensors.helix > HELIX_MAX_ANGLE) {
            ledcAttachPin(PIN_MOTOR_REVERSE_B, 1);
            digitalWrite(PIN_MOTOR_FORWARD_B, HIGH);
            ledcWrite(1, abs(dutyCycle));
        }
    }
}