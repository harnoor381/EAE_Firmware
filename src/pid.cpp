#include "eae/pid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace eae {

Pid::Pid(PidSettings settings) : settings_(settings) {
    for (double value : {settings.kp, settings.ki, settings.kd,
                         settings.derivative_filter_seconds}) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument("PID gains and filter time must be finite and nonnegative");
        }
    }
}

double Pid::update(double setpoint_c, double measured_c, double dt_seconds) {
    if (!std::isfinite(setpoint_c) || !std::isfinite(measured_c) ||
        !std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("PID requires finite temperatures and a positive time step");
    }

    // 1. Calculate how far the measured temperature is above the target.
    const double error = measured_c - setpoint_c;
    const double proportional = settings_.kp * error;

    // 2. Measure temperature change. Using the measurement avoids a sudden
    // derivative spike when only the setpoint changes. Filter small changes.
    double derivative = 0.0;
    if (have_previous_measurement_) {
        derivative = (measured_c - previous_measurement_) / dt_seconds;
    }
    const double alpha = dt_seconds /
        (settings_.derivative_filter_seconds + dt_seconds);
    const double filtered_derivative = filtered_derivative_ +
        alpha * (derivative - filtered_derivative_);
    const double derivative_term = settings_.kd * filtered_derivative;

    // 3. Try adding this sample to the integral term. Stop adding when it
    // would push an already limited output farther beyond 0% or 100%.
    const double candidate_integral = integral_ + settings_.ki * error * dt_seconds;
    const double candidate_output = proportional + candidate_integral + derivative_term;
    for (double value : {error, proportional, derivative, filtered_derivative,
                         derivative_term, candidate_integral, candidate_output}) {
        if (!std::isfinite(value)) {
            throw std::overflow_error("PID calculation overflow: reduce gains or check input values");
        }
    }
    filtered_derivative_ = filtered_derivative;
    previous_measurement_ = measured_c;
    have_previous_measurement_ = true;

    const bool pushing_above_maximum = candidate_output > 100.0 && error > 0.0;
    const bool pushing_below_minimum = candidate_output < 0.0 && error < 0.0;
    if (pushing_above_maximum) {
        // Allow the integral to reach the limit, but not pass it. This also
        // handles a large integral step when the proportional gain is zero.
        integral_ = std::max(integral_, 100.0 - proportional - derivative_term);
    } else if (pushing_below_minimum) {
        integral_ = std::min(integral_, -proportional - derivative_term);
    } else {
        integral_ = candidate_integral;
    }

    // 4. Keep the final logical fan command within its allowed range.
    return std::clamp(proportional + integral_ + derivative_term, 0.0, 100.0);
}

void Pid::reset() {
    integral_ = 0.0;
    previous_measurement_ = 0.0;
    filtered_derivative_ = 0.0;
    have_previous_measurement_ = false;
}

}  // namespace eae
