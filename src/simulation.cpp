#include "eae/simulation.hpp"
#include "eae/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace eae {

const char* name(Scenario scenario) {
    switch (scenario) {
        case Scenario::Nominal: return "nominal";
        case Scenario::HeatLoad: return "heat-load";
        case Scenario::PumpTimeout: return "pump-timeout";
        case Scenario::LowCoolant: return "low-coolant";
        case Scenario::SensorFault: return "sensor-fault";
        case Scenario::OverTemperature: return "overtemperature";
        case Scenario::PumpFault: return "pump-fault";
    }
    return "unknown";
}

void validate(const SimulationOptions& options) {
    validate(options.config);
    if (!std::isfinite(options.duration_seconds) || options.duration_seconds <= 0.0 ||
        options.duration_seconds > 86400.0) {
        throw std::invalid_argument("Duration must be greater than 0 and at most 86400 seconds");
    }
    if (!std::isfinite(options.step_seconds) || options.step_seconds <= 0.0 ||
        options.step_seconds > 0.25 ||
        options.duration_seconds / options.step_seconds > 2000000.0) {
        throw std::invalid_argument("Time step must be in (0, 0.25] seconds, with at most 2000000 steps");
    }
    if (!std::isfinite(options.initial_temperature_c) ||
        options.initial_temperature_c < options.config.minimum_sensor_c ||
        options.initial_temperature_c > options.config.maximum_sensor_c) {
        throw std::invalid_argument("Initial temperature must be inside the sensor table range");
    }
    if (!std::isfinite(options.print_interval_seconds) || options.print_interval_seconds <= 0.0) {
        throw std::invalid_argument("Print interval must be positive and finite");
    }
    // Reject an enum value that did not come from the supported scenario list.
    if (options.scenario < Scenario::Nominal || options.scenario > Scenario::PumpFault) {
        throw std::invalid_argument("Unknown simulation scenario");
    }
}

ThermalModel::ThermalModel(double initial_c, int maximum_pump_rpm, double ambient_c)
    : temperature_c_(initial_c), maximum_pump_rpm_(maximum_pump_rpm), ambient_c_(ambient_c) {
    if (!std::isfinite(initial_c) || !std::isfinite(ambient_c) || maximum_pump_rpm <= 0) {
        throw std::invalid_argument("Invalid thermal model settings");
    }
}

void ThermalModel::set_temperature(double temperature_c) {
    if (!std::isfinite(temperature_c)) {
        throw std::invalid_argument("Physical model temperature must be finite");
    }
    temperature_c_ = temperature_c;
}

void ThermalModel::update(double dt_seconds, double heat_input_w,
                          double actual_pump_rpm, double fan_command_percent) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0 ||
        !std::isfinite(heat_input_w) || heat_input_w < 0.0 ||
        !std::isfinite(actual_pump_rpm) || actual_pump_rpm < 0.0 ||
        actual_pump_rpm > static_cast<double>(maximum_pump_rpm_) ||
        !std::isfinite(fan_command_percent) || fan_command_percent < 0.0 ||
        fan_command_percent > 100.0) {
        throw std::invalid_argument("Invalid thermal model input");
    }

    // Give the fan a one-second response time instead of instant speed changes.
    fan_percent_ += (1.0 - std::exp(-dt_seconds)) * (fan_command_percent - fan_percent_);

    // Simple demonstration model: circulating coolant and radiator airflow
    // increase heat removal. These coefficients are not measured WP32/SPAL data.
    constexpr double heat_capacity_j_per_c = 4000.0;
    const double pump_fraction = actual_pump_rpm / static_cast<double>(maximum_pump_rpm_);
    const double conductance_w_per_c = 2.0 + pump_fraction * (4.0 + 0.96 * fan_percent_);
    const double equilibrium_c = ambient_c_ + heat_input_w / conductance_w_per_c;
    temperature_c_ = equilibrium_c + (temperature_c_ - equilibrium_c) *
        std::exp(-conductance_w_per_c * dt_seconds / heat_capacity_j_per_c);
}

RunSummary run_simulation(const SimulationOptions& options, SampleObserver on_sample,
                          FrameObserver on_frame) {
    validate(options);
    CoolingController controller(options.config);
    SimulatedCanBus bus;
    PumpFeedbackMonitor feedback(options.config.communication_timeout_seconds);
    SimulatedPump pump(options.config.maximum_pump_rpm,
                       options.config.communication_timeout_seconds);
    ThermalModel thermal(options.initial_temperature_c, options.config.maximum_pump_rpm);
    RunSummary summary;
    summary.peak_temperature_c = thermal.temperature_c();

    double now = 0.0;
    if (on_frame) {
        bus.set_observer([&](Direction direction, const CanFrame& frame, bool delivered) {
            on_frame(now, direction, frame, delivered);
        });
    }
    bool overtemperature_injected = false;
    const auto steps = static_cast<std::size_t>(
        std::ceil(options.duration_seconds / options.step_seconds));

    for (std::size_t index = 0; index < steps; ++index) {
        now = static_cast<double>(index) * options.step_seconds;
        const double dt = std::min(options.step_seconds, options.duration_seconds - now);
        if (dt <= 0.0) break;

        // 1. Apply the selected scenario using simulation time.
        const bool ignition = now >= 1.0;
        const bool fault_window = now >= 40.0 && now < 55.0;
        const bool lost_can = options.scenario == Scenario::PumpTimeout && fault_window;
        const bool low_coolant = options.scenario == Scenario::LowCoolant && fault_window;
        const bool sensor_fault = options.scenario == Scenario::SensorFault && fault_window;
        const bool pump_fault = options.scenario == Scenario::PumpFault && fault_window;
        bus.set_drop_messages(lost_can, lost_can);
        if (options.scenario == Scenario::OverTemperature && now >= 40.0 &&
            !overtemperature_injected) {
            thermal.set_temperature(97.0);
            overtemperature_injected = true;
        }

        // 2. Let the pump receive the previous scan's command and send feedback.
        // No fabricated 'communication OK' flag is supplied to the controller.
        pump.update(bus, now, dt, ignition, pump_fault);
        while (const auto frame = bus.receive_at_controller()) {
            feedback.receive(*frame, now);
        }
        if (!ignition) feedback.reset();

        // 3. Read inputs. Sensor failure changes the measured value only;
        // the simulated physical coolant temperature still exists.
        Inputs inputs;
        inputs.ignition = ignition;
        inputs.coolant_level_ok = !low_coolant;
        inputs.temperature_c = sensor_fault ? std::numeric_limits<double>::quiet_NaN()
                                            : thermal.temperature_c();
        inputs.temperature_sensor_ok = !sensor_fault;
        inputs.pump_communication_ok = feedback.communication_ok(now);
        inputs.pump_reported_fault = feedback.feedback().device_fault;
        const Outputs outputs = controller.update(inputs, dt);

        // 4. Send a real encoded frame through the simulated queue.
        if (ignition) bus.send_to_pump(encode_pump_command(outputs.pump_rpm));

        // 5. In this demo, the load obeys the simulated shutdown request.
        // Increase heat input between 60 and 150 seconds in the heat-load test.
        const bool high_load = options.scenario == Scenario::HeatLoad &&
            now >= 60.0 && now < 150.0;
        const double heat_w = outputs.shutdown_requested ? 0.0 : (high_load ? 3000.0 : 2000.0);

        const Sample sample{now, thermal.temperature_c(), inputs, outputs,
                            pump.actual_rpm(), feedback.feedback().rpm,
                            thermal.fan_percent(), heat_w};
        if (on_sample) on_sample(sample);
        ++summary.samples;
        ++summary.state_samples[static_cast<std::size_t>(outputs.state)];
        ++summary.fault_samples[static_cast<std::size_t>(outputs.fault)];
        summary.peak_temperature_c = std::max(summary.peak_temperature_c, thermal.temperature_c());

        // 6. Commands affect the next temperature sample, closing the PID loop.
        thermal.update(dt, heat_w, pump.actual_rpm(), outputs.fan_percent);
    }

    summary.final_temperature_c = thermal.temperature_c();
    summary.peak_temperature_c = std::max(summary.peak_temperature_c, thermal.temperature_c());
    summary.bus = bus.statistics();
    return summary;
}

}  // namespace eae
