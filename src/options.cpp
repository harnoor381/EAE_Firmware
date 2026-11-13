#include "eae/options.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

namespace eae {
namespace {

double number(const std::string& option, const std::string& value) {
    try {
        std::size_t consumed = 0;
        const double result = std::stod(value, &consumed);
        if (consumed == value.size() && std::isfinite(result)) return result;
    } catch (const std::exception&) {
        // Use one clear error message for bad text and numbers outside range.
    }
    throw std::invalid_argument(option + " requires a finite number, received: " + value);
}

Scenario scenario(const std::string& value) {
    for (Scenario candidate : {Scenario::Nominal, Scenario::HeatLoad, Scenario::PumpTimeout,
                               Scenario::LowCoolant, Scenario::SensorFault,
                               Scenario::OverTemperature, Scenario::PumpFault}) {
        if (value == name(candidate)) return candidate;
    }
    throw std::invalid_argument("Unknown scenario: " + value);
}

}  // namespace

CommandLine parse_arguments(const std::vector<std::string>& arguments) {
    CommandLine result;
    std::set<std::string> seen;
    for (std::size_t i = 0; i < arguments.size(); ++i) {
        const std::string& key = arguments[i];
        if (!seen.insert(key).second) throw std::invalid_argument("Repeated option: " + key);
        if (key == "--help" || key == "-h") {
            result.help = true;
            continue;
        }
        if (key == "--can-trace") {
            result.trace_can = true;
            continue;
        }
        const std::set<std::string> values{
            "--setpoint", "--kp", "--ki", "--kd", "--duration", "--dt",
            "--start-temp", "--print-every", "--scenario", "--csv",
            "--warning-on", "--warning-off", "--critical-on", "--critical-off",
            "--can-timeout"
        };
        if (values.count(key) == 0) throw std::invalid_argument("Unknown option: " + key);
        if (i + 1 >= arguments.size() || arguments[i + 1].rfind("--", 0) == 0) {
            throw std::invalid_argument("Missing value for " + key);
        }
        const std::string& value = arguments[++i];
        if (key == "--scenario") result.simulation.scenario = scenario(value);
        else if (key == "--csv") {
            if (value.empty()) throw std::invalid_argument("CSV path cannot be empty");
            result.csv_path = value;
        } else {
            const double parsed = number(key, value);
            auto& sim = result.simulation;
            if (key == "--setpoint") sim.config.setpoint_c = parsed;
            else if (key == "--kp") sim.config.pid.kp = parsed;
            else if (key == "--ki") sim.config.pid.ki = parsed;
            else if (key == "--kd") sim.config.pid.kd = parsed;
            else if (key == "--duration") sim.duration_seconds = parsed;
            else if (key == "--dt") sim.step_seconds = parsed;
            else if (key == "--start-temp") sim.initial_temperature_c = parsed;
            else if (key == "--print-every") sim.print_interval_seconds = parsed;
            else if (key == "--warning-on") sim.config.warning_on_c = parsed;
            else if (key == "--warning-off") sim.config.warning_off_c = parsed;
            else if (key == "--critical-on") sim.config.critical_on_c = parsed;
            else if (key == "--critical-off") sim.config.critical_off_c = parsed;
            else if (key == "--can-timeout") sim.config.communication_timeout_seconds = parsed;
        }
    }
    validate(result.simulation);
    return result;
}

std::string help_text() {
    return R"(EAE_Firmware: cooling control and CAN simulation

Usage: eae_firmware [options]
  --setpoint C        PID coolant target (default 75)
  --kp VALUE          Proportional gain (default 10)
  --ki VALUE          Integral gain (default 0.2)
  --kd VALUE          Derivative gain (default 2)
  --scenario NAME     nominal, heat-load (default), pump-timeout, low-coolant,
                      sensor-fault, overtemperature, or pump-fault
  --duration SECONDS  Simulation duration (default 240)
  --dt SECONDS        Control time step, greater than 0 and at most 0.25 (default 0.1)
  --start-temp C      Initial coolant temperature (default 70)
  --warning-on C      Start warning at this temperature (default 90)
  --warning-off C     Clear warning strictly below this temperature (default 87)
  --critical-on C     Start critical fault at this temperature (default 95)
  --critical-off C    Clear critical fault strictly below this temperature (default 90)
  --can-timeout SEC   Pump command and feedback timeout (default 0.5)
  --print-every SEC   Console sample interval (default 5); transitions also print
  --can-trace         Print encoded CAN frames and dropped messages
  --csv PATH          Write every simulation sample to a CSV file
  --help, -h          Show this message

All parameters are demonstration settings. No hardware is connected.
Simulation time advances without real-time delays.
)";
}

}  // namespace eae
