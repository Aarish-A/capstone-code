#include "state.h"

ECVTState updateState(ECVTState currentState) {
    ECVTState nextState = currentState;

    // TODO:
    //  - Sensitivity for throttle, brake, etc.

    switch (currentState) {
        case Idle:
            // TODO: Switching back to idle logic, need vehicle speed for that
            if (sensors.throttle > 0) {
                nextState = Drive;
            }
            break;
        case Brake:
            // TODO: Switching to Brake state logic
            break;
        case Drive:
            // TODO: Switching back to idle logic, need vehicle speed for that
            break;
    }

    return nextState;
}