#pragma once

namespace eae {

enum class State { Off, Normal, Warning, Fault };
enum class Fault {
    None, LowCoolant, PumpCommunication, PumpDevice,
    TemperatureSensor, OverTemperature
};

struct PidSettings {
    double kp = 10.0;
    double ki = 0.2;
    double kd = 2.0;
    double derivative_filter_seconds = 0.5;
};

struct Config {
    // Demonstration values. Confirm the real equipment settings before use.
    int minimum_pump_rpm = 1500;
    int maximum_pump_rpm = 4600;
    double setpoint_c = 75.0;
    double warning_on_c = 90.0;
    double warning_off_c = 87.0;
    double critical_on_c = 95.0;
    double critical_off_c = 90.0;
    double minimum_sensor_c = -20.0;
    double maximum_sensor_c = 160.0;
    double communication_timeout_seconds = 0.5;
    PidSettings pid;
};

struct Inputs {
    bool ignition = false;
    bool coolant_level_ok = true;
    double temperature_c = 25.0;
    bool temperature_sensor_ok = true;
    bool pump_communication_ok = false;
    bool pump_reported_fault = false;
};

struct Outputs {
    State state = State::Off;
    Fault fault = Fault::None;
    int pump_rpm = 0;
    // A logical cooling command, not a verified electrical PWM duty cycle.
    double fan_percent = 0.0;
    // This request is simulated. It does not stop any physical equipment.
    bool shutdown_requested = true;
};

const char* name(State state);
const char* name(Fault fault);
void validate(const Config& config);

}  // namespace eae
