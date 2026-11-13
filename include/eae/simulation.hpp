#pragma once

#include "eae/can.hpp"
#include "eae/types.hpp"

#include <array>
#include <cstddef>
#include <functional>

namespace eae {

enum class Scenario {
    Nominal, HeatLoad, PumpTimeout, LowCoolant,
    SensorFault, OverTemperature, PumpFault
};

const char* name(Scenario scenario);

struct SimulationOptions {
    Config config;
    Scenario scenario = Scenario::HeatLoad;
    double duration_seconds = 240.0;
    double step_seconds = 0.1;
    double initial_temperature_c = 70.0;
    double print_interval_seconds = 5.0;
};

void validate(const SimulationOptions& options);

class ThermalModel {
public:
    ThermalModel(double initial_c, int maximum_pump_rpm, double ambient_c = 25.0);
    void update(double dt_seconds, double heat_input_w,
                double actual_pump_rpm, double fan_command_percent);
    void set_temperature(double temperature_c);
    double temperature_c() const { return temperature_c_; }
    double fan_percent() const { return fan_percent_; }

private:
    double temperature_c_;
    int maximum_pump_rpm_;
    double ambient_c_;
    double fan_percent_ = 0.0;
};

struct Sample {
    double time_seconds = 0.0;
    double actual_temperature_c = 0.0;
    Inputs inputs;
    Outputs outputs;
    double actual_pump_rpm = 0.0;
    int received_pump_rpm = 0;
    double actual_fan_percent = 0.0;
    double heat_input_w = 0.0;
};

struct RunSummary {
    std::size_t samples = 0;
    double final_temperature_c = 0.0;
    double peak_temperature_c = 0.0;
    std::array<std::size_t, 4> state_samples{};
    std::array<std::size_t, 6> fault_samples{};
    BusStatistics bus;
};

using SampleObserver = std::function<void(const Sample&)>;
using FrameObserver = std::function<void(double, Direction, const CanFrame&, bool)>;

// Use simulation time rather than sleeping. Repeated runs are deterministic.
RunSummary run_simulation(const SimulationOptions& options,
                          SampleObserver on_sample = {},
                          FrameObserver on_frame = {});

}  // namespace eae
