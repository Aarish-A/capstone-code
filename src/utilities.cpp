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
