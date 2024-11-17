#include "drone_state.hpp"

#ifndef DRONE_CONSTANTS_HPP
#define DRONE_CONSTANTS_HPP

namespace DroneConstants {
    constexpr double DEFAULT_STATE = DroneState::IDLE;
    constexpr double TARGET_ALTITUDE = 15.0;
    constexpr double DRONE_SPEED = 5.0;
    constexpr double POSITION_MARGIN = 19.5; // margin of error for distance to target
    constexpr double ALTITUDE_MARGIN = 1.0; // margin of error for altitude
}

#endif