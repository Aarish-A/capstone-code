#include "motors.h"

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

void setMotor(float dutyCycle, SensorValues *sensors) {
    bool reverse = dutyCycle > 0 ? true : false;

    digitalWrite(PIN_MOTOR_FORWARD_A, LOW);
    ledcWrite(0, 0);  // Channel 0 = PIN_MOTOR_FORWARD_B
    digitalWrite(PIN_MOTOR_REVERSE_A, LOW);
    ledcWrite(1, 0);  // Channel 1 = PIN_MOTOR_REVERSE_B

    delay(1);

    if (!reverse) {
        if (sensors->helix < HELIX_MAX_ANGLE) {
            digitalWrite(PIN_MOTOR_FORWARD_A, HIGH);
            ledcWrite(0, (uint32_t)(dutyCycle * 100));
        }
    } else if (reverse) {
        if (sensors->helix > HELIX_MAX_ANGLE) {
            digitalWrite(PIN_MOTOR_FORWARD_B, HIGH);
            ledcWrite(1, (uint32_t)(dutyCycle * 100));
        }
    }
}