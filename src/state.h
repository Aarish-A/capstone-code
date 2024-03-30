#pragma once

#include "sensors.h"

typedef enum {
    Idle,
    Brake,
    Drive,
} ECVTState;

void initStateMachine();  // Initialize state machine
ECVTState updateState();  // Update ECVT State based on current state

// Global Extern Variables
extern ECVTState currentState;