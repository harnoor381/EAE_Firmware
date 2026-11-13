#include "eae/controller.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace {

eae::Inputs healthy(double temperature = 75.0) {
    return {true, true, temperature, true, true, false};
}

TEST(Controller, IgnitionOffOverridesEveryFault) {
    eae::CoolingController controller;
    auto input = healthy(100.0);
    input.ignition = false;
    input.coolant_level_ok = false;
    input.pump_communication_ok = false;
    input.temperature_sensor_ok = false;
    const auto out = controller.update(input, 0.1);
    EXPECT_EQ(out.state, eae::State::Off);
    EXPECT_EQ(out.fault, eae::Fault::None);
    EXPECT_EQ(out.pump_rpm, 0);
    EXPECT_DOUBLE_EQ(out.fan_percent, 0.0);
    EXPECT_TRUE(out.shutdown_requested);
}

TEST(Controller, HotStartupProtectsDuringFirstCall) {
    eae::CoolingController controller;
    const auto out = controller.update(healthy(97.0), 0.1);
    EXPECT_EQ(out.fault, eae::Fault::OverTemperature);
    EXPECT_EQ(out.pump_rpm, 4600);
    EXPECT_DOUBLE_EQ(out.fan_percent, 100.0);
    EXPECT_TRUE(out.shutdown_requested);
}

TEST(Controller, LowCoolantHasHighestFaultPriority) {
    eae::CoolingController controller;
    auto input = healthy(97.0);
    input.coolant_level_ok = false;
    input.pump_communication_ok = false;
    input.temperature_sensor_ok = false;
    const auto out = controller.update(input, 0.1);
    EXPECT_EQ(out.fault, eae::Fault::LowCoolant);
    EXPECT_EQ(out.pump_rpm, 0);
    EXPECT_DOUBLE_EQ(out.fan_percent, 0.0);
    EXPECT_TRUE(out.shutdown_requested);
}

TEST(Controller, LostCommunicationOverridesSensorFailure) {
    eae::CoolingController controller;
    auto input = healthy(std::numeric_limits<double>::quiet_NaN());
    input.temperature_sensor_ok = false;
    input.pump_communication_ok = false;
    const auto out = controller.update(input, 0.1);
    EXPECT_EQ(out.fault, eae::Fault::PumpCommunication);
    EXPECT_EQ(out.pump_rpm, 0);
    EXPECT_DOUBLE_EQ(out.fan_percent, 100.0);
    EXPECT_TRUE(out.shutdown_requested);
}

TEST(Controller, PumpCanReportFaultWithHealthyCommunication) {
    eae::CoolingController controller;
    auto input = healthy();
    input.pump_reported_fault = true;
    const auto out = controller.update(input, 0.1);
    EXPECT_EQ(out.fault, eae::Fault::PumpDevice);
    EXPECT_EQ(out.pump_rpm, 0);
    EXPECT_TRUE(out.shutdown_requested);
}

class InvalidTemperature : public testing::TestWithParam<double> {};
TEST_P(InvalidTemperature, RequestsCoolingAndShutdown) {
    eae::CoolingController controller;
    const auto out = controller.update(healthy(GetParam()), 0.1);
    EXPECT_EQ(out.fault, eae::Fault::TemperatureSensor);
    EXPECT_EQ(out.pump_rpm, 4600);
    EXPECT_DOUBLE_EQ(out.fan_percent, 100.0);
    EXPECT_TRUE(out.shutdown_requested);
}
INSTANTIATE_TEST_SUITE_P(InvalidReadings, InvalidTemperature, testing::Values(
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), -20.01, 160.01));

TEST(Controller, SensorFaultFlagRejectsPlausibleReading) {
    eae::CoolingController controller;
    auto input = healthy();
    input.temperature_sensor_ok = false;
    EXPECT_EQ(controller.update(input, 0.1).fault, eae::Fault::TemperatureSensor);
}

TEST(Controller, WarningUsesExactHysteresisBoundaries) {
    eae::CoolingController controller;
    EXPECT_EQ(controller.update(healthy(89.99), 0.1).state, eae::State::Normal);
    const auto start = controller.update(healthy(90.0), 0.1);
    EXPECT_EQ(start.state, eae::State::Warning);
    EXPECT_FALSE(start.shutdown_requested);
    EXPECT_DOUBLE_EQ(start.fan_percent, 100.0);
    EXPECT_EQ(controller.update(healthy(88.0), 0.1).state, eae::State::Warning);
    EXPECT_EQ(controller.update(healthy(87.0), 0.1).state, eae::State::Warning);
    EXPECT_EQ(controller.update(healthy(86.99), 0.1).state, eae::State::Normal);
}

TEST(Controller, CriticalFaultClearsBelowNinetyAndLeavesWarning) {
    eae::CoolingController controller;
    EXPECT_EQ(controller.update(healthy(95.0), 0.1).fault, eae::Fault::OverTemperature);
    EXPECT_EQ(controller.update(healthy(92.0), 0.1).fault, eae::Fault::OverTemperature);
    EXPECT_EQ(controller.update(healthy(90.0), 0.1).fault, eae::Fault::OverTemperature);
    const auto recovered = controller.update(healthy(89.99), 0.1);
    EXPECT_EQ(recovered.state, eae::State::Warning);
    EXPECT_EQ(recovered.fault, eae::Fault::None);
    EXPECT_FALSE(recovered.shutdown_requested);
}

TEST(Controller, InvalidSensorCannotClearCriticalHistory) {
    eae::CoolingController controller;
    controller.update(healthy(97.0), 0.1);
    auto broken = healthy(25.0);
    broken.temperature_sensor_ok = false;
    EXPECT_EQ(controller.update(broken, 0.1).fault, eae::Fault::TemperatureSensor);
    EXPECT_EQ(controller.update(healthy(92.0), 0.1).fault, eae::Fault::OverTemperature);
}

TEST(Controller, OtherFaultDoesNotEraseTemperatureHistory) {
    eae::CoolingController controller;
    auto input = healthy(97.0);
    input.coolant_level_ok = false;
    controller.update(input, 0.1);
    EXPECT_EQ(controller.update(healthy(92.0), 0.1).fault, eae::Fault::OverTemperature);
}

TEST(Controller, FaultRecoveryCalculatesCoolingImmediately) {
    eae::CoolingController controller;
    auto input = healthy();
    input.coolant_level_ok = false;
    controller.update(input, 0.1);
    const auto out = controller.update(healthy(), 0.1);
    EXPECT_EQ(out.state, eae::State::Normal);
    EXPECT_EQ(out.pump_rpm, 3500);
    EXPECT_FALSE(out.shutdown_requested);
}

TEST(Controller, PIDMemoryIsResetDuringAnOverride) {
    eae::Config config;
    config.pid = {0.0, 1.0, 0.0, 0.5};
    eae::CoolingController controller(config);
    for (int i = 0; i < 100; ++i) controller.update(healthy(80.0), 0.1);
    EXPECT_GT(controller.update(healthy(80.0), 0.1).fan_percent, 40.0);
    controller.update(healthy(90.0), 0.1);
    EXPECT_NEAR(controller.update(healthy(80.0), 0.1).fan_percent, 0.5, 1e-10);
}

TEST(Controller, IgnitionCycleClearsHistoryAndHotRestartRechecks) {
    eae::CoolingController controller;
    controller.update(healthy(97.0), 0.1);
    controller.update({}, 0.1);
    EXPECT_EQ(controller.update(healthy(88.0), 0.1).state, eae::State::Normal);
    controller.update({}, 0.1);
    EXPECT_EQ(controller.update(healthy(97.0), 0.1).fault, eae::Fault::OverTemperature);
}

class PumpBands : public testing::TestWithParam<std::tuple<double, int>> {};
TEST_P(PumpBands, NormalSpeedMatchesTemperatureBand) {
    eae::CoolingController controller;
    const auto [temperature, expected] = GetParam();
    const auto out = controller.update(healthy(temperature), 0.1);
    EXPECT_EQ(out.state, eae::State::Normal);
    EXPECT_EQ(out.pump_rpm, expected);
    EXPECT_GE(out.fan_percent, 0.0);
    EXPECT_LE(out.fan_percent, 100.0);
}
INSTANTIATE_TEST_SUITE_P(Boundaries, PumpBands, testing::Values(
    std::make_tuple(-20.0, 1500), std::make_tuple(39.99, 1500),
    std::make_tuple(40.0, 2000), std::make_tuple(59.99, 2000),
    std::make_tuple(60.0, 3000), std::make_tuple(69.99, 3000),
    std::make_tuple(70.0, 3500), std::make_tuple(79.99, 3500),
    std::make_tuple(80.0, 4000), std::make_tuple(84.99, 4000),
    std::make_tuple(85.0, 4600), std::make_tuple(89.99, 4600)));

TEST(Controller, CommandsRespectPumpCalibration) {
    eae::Config config;
    config.maximum_pump_rpm = 3200;
    eae::CoolingController controller(config);
    EXPECT_EQ(controller.update(healthy(82.0), 0.1).pump_rpm, 3200);
}

TEST(Controller, InvalidConfigurationAndTimeStepsAreRejected) {
    eae::Config config;
    config.setpoint_c = 90.0;
    EXPECT_THROW(eae::CoolingController{config}, std::invalid_argument);
    config = {};
    config.critical_off_c = 96.0;
    EXPECT_THROW(eae::CoolingController{config}, std::invalid_argument);
    config = {};
    config.maximum_pump_rpm = 1000;
    EXPECT_THROW(eae::CoolingController{config}, std::invalid_argument);
    eae::CoolingController controller;
    EXPECT_THROW(controller.update(healthy(), 0.0), std::invalid_argument);
    EXPECT_THROW(controller.update(healthy(), -0.1), std::invalid_argument);
}

}  // namespace
