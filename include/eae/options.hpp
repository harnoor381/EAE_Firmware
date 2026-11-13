#pragma once

#include "eae/simulation.hpp"

#include <string>
#include <vector>

namespace eae {

struct CommandLine {
    SimulationOptions simulation;
    bool help = false;
    bool trace_can = false;
    std::string csv_path;
};

CommandLine parse_arguments(const std::vector<std::string>& arguments);
std::string help_text();

}  // namespace eae
