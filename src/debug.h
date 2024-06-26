#pragma once

#include <Arduino.h>

#include <functional>

#include "bluetooth.h"

template <typename Func, typename... Args>
unsigned long timeFunctionExecution(std::string functionName, Func func, Args&&... args) {
    unsigned long startTime = micros();
    func(std::forward<Args>(args)...);  // Execute the function with its arguments
    unsigned long endTime = micros();

    unsigned long executionTime = (micros() - startTime);
    logBluetooth((functionName + " Execution time (micros)").c_str(), executionTime);
    return executionTime;  // Return execution time in microseconds
}