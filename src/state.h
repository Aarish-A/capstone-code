#pragma once

#include "sensors.h"

typedef enum {
    Idle,
    Brake,
    Drive,
} ECVTState;

ECVTState updateState(ECVTState currentState, SensorValues *sensors);