#include "eae/controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace eae {

const char* name(State state) {
    switch (state) {
        case State::Off: return "OFF";
        case State::Normal: return "NORMAL";
        case State::Warning: return "WARNING";
        case State::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

const char* name(Fault fault) {
    switch (fault) {
        case Fault::None: return "NONE";
        case Fault::LowCoolant: return "LOW_COOLANT";
        case Fault::PumpCommunication: return "PUMP_COMMUNICATION";
        case Fault::PumpDevice: return "PUMP_DEVICE";
        case Fault::TemperatureSensor: return "TEMP_SENSOR";
        case Fault::OverTemperature: return "OVER_TEMPERATURE";
    }
    return "UNKNOWN";
}

void validate(const Config& config) {
    for (double value : {config.setpoint_c, config.warning_on_c, config.warning_off_c,
                         config.critical_on_c, config.critical_off_c,
                         config.minimum_sensor_c, config.maximum_sensor_c,
                         config.communication_timeout_seconds}) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("Controller settings must be finite numbers");
        }
    }
    if (!(config.minimum_sensor_c < config.setpoint_c &&
          config.setpoint_c < config.warning_off_c &&
          config.warning_off_c < config.warning_on_c &&
          config.warning_on_c <= config.critical_off_c &&
          config.critical_off_c < config.critical_on_c &&
          config.critical_on_c <= config.maximum_sensor_c)) {
        throw std::invalid_argument("Temperature setpoints and hysteresis limits are inconsistent");
    }
    if (config.minimum_pump_rpm <= 0 ||
        config.maximum_pump_rpm < config.minimum_pump_rpm ||
        config.maximum_pump_rpm > 65535) {
        throw std::invalid_argument("Pump calibration must satisfy 0 < minimum <= maximum <= 65535 RPM");
    }
    if (config.communication_timeout_seconds <= 0.0 ||
        config.communication_timeout_seconds > 10.0) {
        throw std::invalid_argument("Communication timeout must be greater than 0 and at most 10 seconds");
    }
    // Check PID settings through the same validation used by the PID class.
    const Pid checked_pid(config.pid);
    (void)checked_pid;
}

CoolingController::CoolingController(Config config)
    : config_(config), pid_(config.pid) {
    validate(config_);
}

Outputs CoolingController::override_outputs(State state, Fault fault, int pump_rpm,
                                             double fan_percent, bool shutdown) {
    // Safety rules take control of the outputs. Clear PID memory while it is
    // inactive so it does not accumulate error during a warning or fault.
    pid_.reset();
    return {state, fault, pump_rpm, fan_percent, shutdown};
}

Outputs CoolingController::update(const Inputs& inputs, double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("Controller time step must be positive and finite");
    }

    // 1. IGNITION OFF: stop cooling and request that the equipment stays off.
    if (!inputs.ignition) {
        warning_active_ = false;
        critical_active_ = false;
        return override_outputs(State::Off, Fault::None, 0, 0.0, true);
    }

    // 2. TEMPERATURE HISTORY: a bad sensor reading cannot clear a warning.
    const bool temperature_valid = inputs.temperature_sensor_ok &&
        std::isfinite(inputs.temperature_c) &&
        inputs.temperature_c >= config_.minimum_sensor_c &&
        inputs.temperature_c <= config_.maximum_sensor_c;
    if (temperature_valid) {
        if (inputs.temperature_c >= config_.warning_on_c) {
            warning_active_ = true;
        } else if (inputs.temperature_c < config_.warning_off_c) {
            warning_active_ = false;
        }
        if (inputs.temperature_c >= config_.critical_on_c) {
            critical_active_ = true;
        } else if (inputs.temperature_c < config_.critical_off_c) {
            critical_active_ = false;
        }
    }

    // Check faults in priority order. Only the first matching fault is shown.

    // 3. LOW COOLANT: avoid requesting pump operation without coolant.
    if (!inputs.coolant_level_ok) {
        return override_outputs(State::Fault, Fault::LowCoolant, 0, 0.0, true);
    }

    // 4. LOST COMMUNICATION: request pump OFF and fan maximum. A zero command
    // does not prove that the pump received it or that the pump has stopped.
    if (!inputs.pump_communication_ok) {
        return override_outputs(State::Fault, Fault::PumpCommunication, 0, 100.0, true);
    }

    // A pump can report a device fault while CAN communication is still healthy.
    if (inputs.pump_reported_fault) {
        return override_outputs(State::Fault, Fault::PumpDevice, 0, 100.0, true);
    }

    // 5. SENSOR FAILURE: the true temperature is unknown. Cool at maximum and
    // request shutdown. Level and pump communication have already passed.
    if (!temperature_valid) {
        return override_outputs(State::Fault, Fault::TemperatureSensor,
                                config_.maximum_pump_rpm, 100.0, true);
    }

    // 6. CRITICAL TEMPERATURE: keep cooling and shutdown active until below
    // the critical reset temperature, 90 C with the demonstration settings.
    if (critical_active_) {
        return override_outputs(State::Fault, Fault::OverTemperature,
                                config_.maximum_pump_rpm, 100.0, true);
    }

    // 7. WARNING: maximum cooling from 90 C until strictly below 87 C.
    if (warning_active_) {
        return override_outputs(State::Warning, Fault::None,
                                config_.maximum_pump_rpm, 100.0, false);
    }

    // 8. NORMAL: keep coolant circulating and use PID to control the fan.
    // Startup and fault recovery calculate commands during this same call.
    return {State::Normal, Fault::None, normal_pump_rpm(inputs.temperature_c),
            pid_.update(config_.setpoint_c, inputs.temperature_c, dt_seconds), false};
}

int CoolingController::normal_pump_rpm(double temperature_c) const {
    int requested = config_.maximum_pump_rpm;
    if (temperature_c < 40.0) requested = 1500;
    else if (temperature_c < 60.0) requested = 2000;
    else if (temperature_c < 70.0) requested = 3000;
    else if (temperature_c < 80.0) requested = 3500;
    else if (temperature_c < 85.0) requested = 4000;
    return std::clamp(requested, config_.minimum_pump_rpm, config_.maximum_pump_rpm);
}

}  // namespace eae
