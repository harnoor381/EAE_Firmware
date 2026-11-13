#pragma once

#include "eae/types.hpp"

namespace eae {

class Pid {
public:
    explicit Pid(PidSettings settings = {});

    // A temperature above the setpoint produces a positive cooling request.
    double update(double setpoint_c, double measured_c, double dt_seconds);
    void reset();
    double integral_term() const { return integral_; }

private:
    PidSettings settings_;
    double integral_ = 0.0;
    double previous_measurement_ = 0.0;
    double filtered_derivative_ = 0.0;
    bool have_previous_measurement_ = false;
};

}  // namespace eae
