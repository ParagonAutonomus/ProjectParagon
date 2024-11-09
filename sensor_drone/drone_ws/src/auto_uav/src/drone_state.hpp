#ifndef DRONE_STATE_HPP
#define DRONE_STATE_HPP

enum DroneState {
    IDLE = 0,
    TAKEOFF = 1,
    READY = 2,
    MOVING = 3,
    WAITING = 4,
    LANDING = 5
};

inline std::string drone_state_to_string(DroneState state) {
    switch (state) {
        case DroneState::IDLE:
            return "IDLE";
        case DroneState::TAKEOFF:
            return "TAKEOFF";
        case DroneState::READY:
            return "READY";
        case DroneState::MOVING:
            return "MOVING";
        case DroneState::WAITING:
            return "WAITING";
        case DroneState::LANDING:
            return "LANDING";
        default:
            return "UNKNOWN";
    }
}

#endif