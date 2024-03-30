#pragma once

float mapFloat(float val, float inputMin, float inputMax, float outputMin, float outputMax);  // Map a float value from one range to another
int mapThrottleToRPM(float throttle);                                                         // Map throttle range [0-1] to optimal Engine RPM