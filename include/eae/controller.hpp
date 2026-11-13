#pragma once

#include "eae/pid.hpp"

namespace eae {

class CoolingController {
public:
    explicit CoolingController(Config config = {});
    Outputs update(const Inputs& inputs, double dt_seconds);

private:
    Outputs override_outputs(State state, Fault fault, int pump_rpm,
                             double fan_percent, bool shutdown);
    int normal_pump_rpm(double temperature_c) const;

    Config config_;
    Pid pid_;
    bool warning_active_ = false;
    bool critical_active_ = false;
};

}  // namespace eae
