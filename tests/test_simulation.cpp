#include "eae/simulation.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace {

TEST(Simulation, NominalLoopRegulatesNearSetpoint) {
    eae::SimulationOptions options;
    options.scenario = eae::Scenario::Nominal;
    const auto summary = eae::run_simulation(options);
    EXPECT_NEAR(summary.final_temperature_c, options.config.setpoint_c, 1.0);
    EXPECT_LT(summary.peak_temperature_c, options.config.warning_on_c);
    EXPECT_EQ(summary.state_samples[static_cast<std::size_t>(eae::State::Fault)], 0U);
    EXPECT_EQ(summary.samples, 2400U);
    EXPECT_GT(summary.bus.commands_sent, 2000U);
    EXPECT_GT(summary.bus.feedback_sent, 2000U);
}

TEST(Simulation, FanRespondsToHeatLoadAndTemperatureRecovers) {
    eae::SimulationOptions options;
    options.scenario = eae::Scenario::HeatLoad;
    double before = 0.0;
    double loaded = 0.0;
    double after = 0.0;
    int before_count = 0, loaded_count = 0, after_count = 0;
    const auto summary = eae::run_simulation(options, [&](const eae::Sample& sample) {
        if (sample.time_seconds >= 40.0 && sample.time_seconds < 55.0) {
            before += sample.outputs.fan_percent;
            ++before_count;
        }
        if (sample.time_seconds >= 125.0 && sample.time_seconds < 145.0) {
            loaded += sample.outputs.fan_percent;
            ++loaded_count;
        }
        if (sample.time_seconds >= 220.0) {
            after += sample.outputs.fan_percent;
            ++after_count;
        }
    });
    ASSERT_GT(before_count, 0);
    ASSERT_GT(loaded_count, 0);
    ASSERT_GT(after_count, 0);
    EXPECT_GT(loaded / loaded_count, before / before_count + 10.0);
    EXPECT_GT(loaded / loaded_count, after / after_count + 10.0);
    EXPECT_NEAR(summary.final_temperature_c, options.config.setpoint_c, 2.0);
    EXPECT_LT(summary.peak_temperature_c, options.config.warning_on_c);
    EXPECT_EQ(summary.state_samples[static_cast<std::size_t>(eae::State::Fault)], 0U);
}

class InjectedFault : public testing::TestWithParam<std::tuple<eae::Scenario, eae::Fault>> {};
TEST_P(InjectedFault, FaultIsDetectedAndThenClears) {
    const auto [scenario, expected_fault] = GetParam();
    eae::SimulationOptions options;
    options.scenario = scenario;
    bool detected = false;
    bool recovered = false;
    const auto summary = eae::run_simulation(options, [&](const eae::Sample& sample) {
        if (sample.outputs.fault == expected_fault) {
            detected = true;
            EXPECT_TRUE(sample.outputs.shutdown_requested);
            EXPECT_EQ(sample.outputs.state, eae::State::Fault);
        }
        if (sample.time_seconds > 100.0 && sample.outputs.state == eae::State::Normal) {
            recovered = true;
        }
        EXPECT_GE(sample.outputs.pump_rpm, 0);
        EXPECT_LE(sample.outputs.pump_rpm, options.config.maximum_pump_rpm);
        EXPECT_GE(sample.outputs.fan_percent, 0.0);
        EXPECT_LE(sample.outputs.fan_percent, 100.0);
        EXPECT_TRUE(std::isfinite(sample.actual_temperature_c));
    });
    EXPECT_TRUE(detected);
    EXPECT_TRUE(recovered);
    EXPECT_GT(summary.fault_samples[static_cast<std::size_t>(expected_fault)], 0U);
}
INSTANTIATE_TEST_SUITE_P(Scenarios, InjectedFault, testing::Values(
    std::make_tuple(eae::Scenario::PumpTimeout, eae::Fault::PumpCommunication),
    std::make_tuple(eae::Scenario::LowCoolant, eae::Fault::LowCoolant),
    std::make_tuple(eae::Scenario::SensorFault, eae::Fault::TemperatureSensor),
    std::make_tuple(eae::Scenario::OverTemperature, eae::Fault::OverTemperature),
    std::make_tuple(eae::Scenario::PumpFault, eae::Fault::PumpDevice)));

TEST(Simulation, CANFaultComesFromElapsedTimeAndRecoversWithFeedback) {
    eae::SimulationOptions options;
    options.scenario = eae::Scenario::PumpTimeout;
    double first_fault = -1.0;
    bool recovered = false;
    const auto summary = eae::run_simulation(options, [&](const eae::Sample& sample) {
        if (sample.outputs.fault == eae::Fault::PumpCommunication && first_fault < 0.0) {
            first_fault = sample.time_seconds;
        }
        if (sample.time_seconds >= 55.0 && sample.time_seconds < 55.2) {
            EXPECT_TRUE(sample.inputs.pump_communication_ok);
            EXPECT_EQ(sample.outputs.state, eae::State::Normal);
            recovered = true;
        }
    });
    EXPECT_GE(first_fault, 40.4);
    EXPECT_LE(first_fault, 40.6);
    EXPECT_TRUE(recovered);
    EXPECT_GT(summary.bus.commands_dropped, 0U);
    EXPECT_GT(summary.bus.feedback_dropped, 0U);
}

TEST(Simulation, SensorFailureDoesNotErasePhysicalTemperature) {
    eae::SimulationOptions options;
    options.scenario = eae::Scenario::SensorFault;
    bool observed = false;
    eae::run_simulation(options, [&](const eae::Sample& sample) {
        if (sample.time_seconds >= 40.0 && sample.time_seconds < 55.0) {
            observed = true;
            EXPECT_TRUE(std::isnan(sample.inputs.temperature_c));
            EXPECT_TRUE(std::isfinite(sample.actual_temperature_c));
            EXPECT_EQ(sample.outputs.fault, eae::Fault::TemperatureSensor);
            EXPECT_EQ(sample.outputs.pump_rpm, 4600);
        }
    });
    EXPECT_TRUE(observed);
}

TEST(Simulation, FrameObserverSeesBothDirectionsAndDrops) {
    eae::SimulationOptions options;
    options.scenario = eae::Scenario::PumpTimeout;
    options.duration_seconds = 56.0;
    std::size_t commands = 0, feedback = 0, dropped = 0;
    const auto result = eae::run_simulation(options, {},
        [&](double now, eae::Direction direction, const eae::CanFrame& frame, bool delivered) {
            EXPECT_GE(now, 0.0);
            EXPECT_LE(frame.size, 8U);
            if (direction == eae::Direction::ToPump) ++commands;
            else ++feedback;
            if (!delivered) ++dropped;
        });
    EXPECT_EQ(commands, result.bus.commands_sent);
    EXPECT_EQ(feedback, result.bus.feedback_sent);
    EXPECT_EQ(dropped, result.bus.commands_dropped + result.bus.feedback_dropped);
}

TEST(Simulation, NonDivisibleDurationUsesFinalPartialStep) {
    eae::SimulationOptions options;
    options.duration_seconds = 2.05;
    options.step_seconds = 0.2;
    double last_time = -1.0;
    const auto result = eae::run_simulation(options, [&](const eae::Sample& sample) {
        EXPECT_LT(sample.time_seconds, options.duration_seconds);
        last_time = sample.time_seconds;
    });
    EXPECT_EQ(result.samples, 11U);
    EXPECT_NEAR(last_time, 2.0, 1e-10);
}

TEST(ThermalModel, MoreCoolingRemovesMoreHeat) {
    eae::ThermalModel low(80.0, 4600);
    eae::ThermalModel high(80.0, 4600);
    for (int i = 0; i < 100; ++i) {
        low.update(0.1, 2000.0, 3500.0, 0.0);
        high.update(0.1, 2000.0, 3500.0, 100.0);
    }
    EXPECT_LT(high.temperature_c(), low.temperature_c());
    EXPECT_GE(high.temperature_c(), 25.0);
}

TEST(ThermalModel, InvalidInputsAreRejected) {
    eae::ThermalModel model(75.0, 4600);
    EXPECT_THROW(model.update(0.0, 2000.0, 3000.0, 50.0), std::invalid_argument);
    EXPECT_THROW(model.update(0.1, 2000.0, 3000.0, 101.0), std::invalid_argument);
    EXPECT_THROW(model.update(0.1, 2000.0, 6000.0, 50.0), std::invalid_argument);
    EXPECT_THROW(model.set_temperature(std::numeric_limits<double>::quiet_NaN()),
                 std::invalid_argument);
}

TEST(Simulation, InvalidRunSettingsAreRejected) {
    eae::SimulationOptions options;
    options.step_seconds = 0.0;
    EXPECT_THROW(eae::run_simulation(options), std::invalid_argument);
    options = {};
    options.step_seconds = 0.5;
    EXPECT_THROW(eae::run_simulation(options), std::invalid_argument);
    options = {};
    options.duration_seconds = -1.0;
    EXPECT_THROW(eae::run_simulation(options), std::invalid_argument);
}

}  // namespace
