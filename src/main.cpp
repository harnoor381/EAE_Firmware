#include "eae/options.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
    try {
        std::vector<std::string> arguments;
        for (int i = 1; i < argc; ++i) arguments.emplace_back(argv[i]);
        const auto options = eae::parse_arguments(arguments);
        if (options.help) {
            std::cout << eae::help_text();
            return 0;
        }

        std::ofstream csv;
        if (!options.csv_path.empty()) {
            csv.exceptions(std::ios::failbit | std::ios::badbit);
            csv.open(options.csv_path);
            csv << "time_s,physical_temp_c,measured_temp_c,setpoint_c,state,fault,"
                   "pump_command_rpm,pump_actual_rpm,pump_received_rpm,"
                   "fan_command_percent,fan_actual_percent,shutdown_requested,"
                   "ignition,level_ok,sensor_ok,pump_comm_ok,heat_input_w\n";
            csv << std::fixed << std::setprecision(6);
        }

        std::cout << "EAE FIRMWARE COOLING SIMULATION\n"
                  << "Scenario: " << eae::name(options.simulation.scenario)
                  << " | Setpoint: " << options.simulation.config.setpoint_c << " C\n"
                  << "Logical commands and simulated feedback; no hardware connected.\n";
        double last_print = -std::numeric_limits<double>::infinity();
        eae::State previous_state = eae::State::Off;
        eae::Fault previous_fault = eae::Fault::None;
        bool first = true;

        const auto on_sample = [&](const eae::Sample& sample) {
            const auto& out = sample.outputs;
            const auto& in = sample.inputs;
            if (csv.is_open()) {
                csv << sample.time_seconds << ',' << sample.actual_temperature_c << ','
                    << in.temperature_c << ',' << options.simulation.config.setpoint_c << ','
                    << eae::name(out.state) << ',' << eae::name(out.fault) << ','
                    << out.pump_rpm << ',' << sample.actual_pump_rpm << ','
                    << sample.received_pump_rpm << ',' << out.fan_percent << ','
                    << sample.actual_fan_percent << ',' << out.shutdown_requested << ','
                    << in.ignition << ',' << in.coolant_level_ok << ','
                    << in.temperature_sensor_ok << ',' << in.pump_communication_ok << ','
                    << sample.heat_input_w << '\n';
            }

            const bool changed = first || out.state != previous_state || out.fault != previous_fault;
            if (changed || sample.time_seconds - last_print >=
                           options.simulation.print_interval_seconds - 1e-12) {
                std::cout << std::fixed << std::setprecision(2)
                          << "t=" << sample.time_seconds << "s"
                          << " | T=" << in.temperature_c << "C"
                          << " | " << eae::name(out.state)
                          << " | fault=" << eae::name(out.fault)
                          << " | pump cmd=" << out.pump_rpm
                          << " actual=" << sample.actual_pump_rpm << "rpm"
                          << " | fan=" << out.fan_percent << "%"
                          << " | shutdown=" << out.shutdown_requested << '\n';
                last_print = sample.time_seconds;
            }
            first = false;
            previous_state = out.state;
            previous_fault = out.fault;
        };

        eae::FrameObserver on_frame;
        if (options.trace_can) {
            on_frame = [](double now, eae::Direction direction,
                          const eae::CanFrame& frame, bool delivered) {
                std::cout << std::fixed << std::setprecision(2) << "CAN t=" << now
                          << (direction == eae::Direction::ToPump ? " TX " : " RX ")
                          << eae::describe(frame) << (delivered ? " QUEUED" : " DROPPED")
                          << '\n';
            };
        }

        const auto summary = eae::run_simulation(options.simulation, on_sample, on_frame);
        if (csv.is_open()) csv.close();
        std::cout << "Finished " << summary.samples << " control steps. Final temperature: "
                  << summary.final_temperature_c << " C; peak: "
                  << summary.peak_temperature_c << " C\n"
                  << "CAN commands: " << summary.bus.commands_sent << " (dropped "
                  << summary.bus.commands_dropped << "); feedback: "
                  << summary.bus.feedback_sent << " (dropped "
                  << summary.bus.feedback_dropped << ")\n";
        if (!options.csv_path.empty()) std::cout << "CSV: " << options.csv_path << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\nUse --help to see valid options.\n";
        return 2;
    }
}
