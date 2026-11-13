#include "eae/options.hpp"

#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

TEST(Options, DefaultsAreValid) {
    const auto parsed = eae::parse_arguments({});
    EXPECT_DOUBLE_EQ(parsed.simulation.config.setpoint_c, 75.0);
    EXPECT_EQ(parsed.simulation.scenario, eae::Scenario::HeatLoad);
    EXPECT_FALSE(parsed.trace_can);
}

TEST(Options, ReadsRuntimeSetpointsGainsAndOutputOptions) {
    const auto parsed = eae::parse_arguments({
        "--setpoint", "72", "--kp", "6", "--ki", "0.3", "--kd", "1",
        "--duration", "120", "--dt", "0.2", "--start-temp", "65",
        "--scenario", "pump-timeout", "--csv", "path with spaces/results.csv",
        "--print-every", "10", "--can-trace", "--can-timeout", "0.8"
    });
    EXPECT_DOUBLE_EQ(parsed.simulation.config.setpoint_c, 72.0);
    EXPECT_DOUBLE_EQ(parsed.simulation.config.pid.kp, 6.0);
    EXPECT_DOUBLE_EQ(parsed.simulation.config.pid.ki, 0.3);
    EXPECT_DOUBLE_EQ(parsed.simulation.config.pid.kd, 1.0);
    EXPECT_DOUBLE_EQ(parsed.simulation.step_seconds, 0.2);
    EXPECT_DOUBLE_EQ(parsed.simulation.config.communication_timeout_seconds, 0.8);
    EXPECT_EQ(parsed.simulation.scenario, eae::Scenario::PumpTimeout);
    EXPECT_EQ(parsed.csv_path, "path with spaces/results.csv");
    EXPECT_TRUE(parsed.trace_can);
}

TEST(Options, ReadsWarningAndCriticalSetpoints) {
    const auto parsed = eae::parse_arguments({
        "--warning-on", "88", "--warning-off", "85",
        "--critical-on", "93", "--critical-off", "89"
    });
    EXPECT_DOUBLE_EQ(parsed.simulation.config.warning_on_c, 88.0);
    EXPECT_DOUBLE_EQ(parsed.simulation.config.warning_off_c, 85.0);
    EXPECT_DOUBLE_EQ(parsed.simulation.config.critical_on_c, 93.0);
    EXPECT_DOUBLE_EQ(parsed.simulation.config.critical_off_c, 89.0);
}

TEST(Options, HelpIsAvailable) {
    EXPECT_TRUE(eae::parse_arguments({"--help"}).help);
    EXPECT_NE(eae::help_text().find("--setpoint"), std::string::npos);
}

class InvalidOptions : public testing::TestWithParam<std::vector<std::string>> {};
TEST_P(InvalidOptions, RejectsInvalidCommandLine) {
    EXPECT_THROW(eae::parse_arguments(GetParam()), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(BadArguments, InvalidOptions, testing::Values(
    std::vector<std::string>{"--unknown"},
    std::vector<std::string>{"--setpoint"},
    std::vector<std::string>{"--setpoint", "--can-trace"},
    std::vector<std::string>{"--setpoint", "nan"},
    std::vector<std::string>{"--setpoint", "inf"},
    std::vector<std::string>{"--setpoint", "75abc"},
    std::vector<std::string>{"--setpoint", "90"},
    std::vector<std::string>{"--kp", "-1"},
    std::vector<std::string>{"--dt", "0"},
    std::vector<std::string>{"--dt", "0.5"},
    std::vector<std::string>{"--duration", "-10"},
    std::vector<std::string>{"--scenario", "unknown"},
    std::vector<std::string>{"--setpoint", "72", "--setpoint", "73"},
    std::vector<std::string>{"--critical-on", "85"},
    std::vector<std::string>{"--can-timeout", "0"},
    std::vector<std::string>{"--csv", ""}));

}  // namespace
