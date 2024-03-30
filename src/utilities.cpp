#include "utilities.h"

float mapFloat(float val, float inputMin, float inputMax, float outputMin, float outputMax) {
    // Ensure the input is within the source range
    val = (val < inputMin) ? inputMin : val;
    val = (val > inputMax) ? inputMax : val;

    // Calculate how 'far' the value is in the source range (0.0 to 1.0)
    float inputRangeScale = (val - inputMin) / (inputMax - inputMin);

    // Map the 'far' value to the target range
    return outputMin + (inputRangeScale * (outputMax - outputMin));
}

int mapThrottleToRPM(float throttle) {
    if (throttle > 0 && throttle <= 0.1)
        return (int)mapFloat(throttle, 0.0, 0.1, 1500, 1600);
    else if (throttle > 0.1 && throttle <= 0.2)
        return (int)mapFloat(throttle, 0.1, 0.2, 1600, 1800);
    else if (throttle > 0.2 && throttle <= 0.3)
        return (int)mapFloat(throttle, 0.2, 0.3, 1800, 2000);
    else if (throttle > 0.3 && throttle <= 0.4)
        return (int)mapFloat(throttle, 0.3, 0.4, 2000, 2200);
    else if (throttle > 0.4 && throttle <= 0.5)
        return (int)mapFloat(throttle, 0.4, 0.5, 2200, 2500);
    else if (throttle > 0.5 && throttle <= 0.6)
        return (int)mapFloat(throttle, 0.5, 0.6, 2500, 2600);
    else if (throttle > 0.6 && throttle <= 0.7)
        return (int)mapFloat(throttle, 0.6, 0.7, 2600, 2700);
    else if (throttle > 0.7 && throttle <= 0.8)
        return (int)mapFloat(throttle, 0.7, 0.8, 2700, 2800);
    else if (throttle > 0.8 && throttle <= 0.9)
        return (int)mapFloat(throttle, 0.8, 0.9, 2800, 2900);
    else if (throttle > 0.9 && throttle <= 1)
        return (int)mapFloat(throttle, 0.9, 1.0, 2800, 3000);

    return 0;
}
