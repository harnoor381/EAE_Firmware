#include "eae/can.hpp"

#include <gtest/gtest.h>
#include <stdexcept>

namespace {

class CanRpm : public testing::TestWithParam<int> {};
TEST_P(CanRpm, CommandAndFeedbackRoundTrip) {
    const auto rpm = GetParam();
    ASSERT_TRUE(eae::decode_pump_command(eae::encode_pump_command(rpm)));
    EXPECT_EQ(*eae::decode_pump_command(eae::encode_pump_command(rpm)), rpm);
    const auto feedback = eae::decode_pump_feedback(eae::encode_pump_feedback({rpm, true}));
    ASSERT_TRUE(feedback);
    EXPECT_EQ(feedback->rpm, rpm);
    EXPECT_TRUE(feedback->device_fault);
}
INSTANTIATE_TEST_SUITE_P(RPMLimits, CanRpm, testing::Values(0, 1500, 3000, 4600, 65535));

TEST(CAN, PayloadUsesLowByteFirst) {
    const auto frame = eae::encode_pump_command(3000);
    EXPECT_EQ(frame.id, 0x200U);
    EXPECT_EQ(frame.size, 2U);
    EXPECT_EQ(frame.data[0], 0xB8U);
    EXPECT_EQ(frame.data[1], 0x0BU);
    EXPECT_EQ(eae::describe(frame), "id=0x200 dlc=2 data=B8 0B");
}

TEST(CAN, MalformedFramesAreRejected) {
    auto frame = eae::encode_pump_command(3000);
    frame.id = 0x123;
    EXPECT_FALSE(eae::decode_pump_command(frame));
    frame = eae::encode_pump_command(3000);
    frame.size = 1;
    EXPECT_FALSE(eae::decode_pump_command(frame));
    frame = eae::encode_pump_feedback({3000, false});
    frame.size = 8;
    EXPECT_FALSE(eae::decode_pump_feedback(frame));
    frame = eae::encode_pump_feedback({3000, false});
    frame.data[2] = 2;
    EXPECT_FALSE(eae::decode_pump_feedback(frame));
    EXPECT_THROW(eae::encode_pump_command(-1), std::invalid_argument);
    EXPECT_THROW(eae::encode_pump_command(65536), std::invalid_argument);
}

TEST(CAN, BothQueuesDeliverFramesInOrder) {
    eae::SimulatedCanBus bus;
    bus.send_to_pump(eae::encode_pump_command(2000));
    bus.send_to_pump(eae::encode_pump_command(3000));
    bus.send_to_controller(eae::encode_pump_feedback({1900, false}));
    EXPECT_EQ(*eae::decode_pump_command(*bus.receive_at_pump()), 2000);
    EXPECT_EQ(*eae::decode_pump_command(*bus.receive_at_pump()), 3000);
    EXPECT_EQ(eae::decode_pump_feedback(*bus.receive_at_controller())->rpm, 1900);
    EXPECT_FALSE(bus.receive_at_pump());
    EXPECT_FALSE(bus.receive_at_controller());
}

TEST(CAN, DroppedFramesAreCountedAndNotDelivered) {
    eae::SimulatedCanBus bus;
    bus.set_drop_messages(true, true);
    EXPECT_FALSE(bus.send_to_pump(eae::encode_pump_command(3000)));
    EXPECT_FALSE(bus.send_to_controller(eae::encode_pump_feedback({3000, false})));
    EXPECT_FALSE(bus.receive_at_pump());
    EXPECT_FALSE(bus.receive_at_controller());
    EXPECT_EQ(bus.statistics().commands_dropped, 1U);
    EXPECT_EQ(bus.statistics().feedback_dropped, 1U);
}

TEST(CAN, QueueCapacityIsBounded) {
    eae::SimulatedCanBus bus(1);
    EXPECT_TRUE(bus.send_to_pump(eae::encode_pump_command(2000)));
    EXPECT_FALSE(bus.send_to_pump(eae::encode_pump_command(3000)));
    EXPECT_EQ(*eae::decode_pump_command(*bus.receive_at_pump()), 2000);
    EXPECT_EQ(bus.statistics().commands_dropped, 1U);
    EXPECT_THROW(eae::SimulatedCanBus{0}, std::invalid_argument);
}

TEST(CAN, FeedbackWatchdogExpiresAfterExactTimeout) {
    eae::PumpFeedbackMonitor monitor(0.5);
    EXPECT_FALSE(monitor.communication_ok(0.0));
    ASSERT_TRUE(monitor.receive(eae::encode_pump_feedback({0, false}), 1.0));
    EXPECT_TRUE(monitor.communication_ok(1.5));
    EXPECT_FALSE(monitor.communication_ok(1.5001));
    EXPECT_FALSE(monitor.communication_ok(0.9));
    monitor.reset();
    EXPECT_FALSE(monitor.communication_ok(1.5));
}

TEST(CAN, MalformedAndOutOfOrderFeedbackDoNotRefreshWatchdog) {
    eae::PumpFeedbackMonitor monitor(0.5);
    const auto valid = eae::encode_pump_feedback({3000, false});
    ASSERT_TRUE(monitor.receive(valid, 1.0));
    EXPECT_FALSE(monitor.receive(eae::encode_pump_command(3000), 1.4));
    EXPECT_FALSE(monitor.receive(valid, 0.9));
    EXPECT_FALSE(monitor.communication_ok(1.6));
}

TEST(CAN, PumpSpeedFollowsCommandThenStopsOnLocalTimeout) {
    eae::SimulatedCanBus bus;
    eae::SimulatedPump pump(4600, 0.5);
    for (int i = 0; i < 30; ++i) {
        bus.send_to_pump(eae::encode_pump_command(3000));
        pump.update(bus, static_cast<double>(i) * 0.1, 0.1, true);
        while (bus.receive_at_controller()) {}
    }
    EXPECT_NEAR(pump.actual_rpm(), 3000.0, 2.0);
    EXPECT_EQ(pump.target_rpm(), 3000);
    pump.update(bus, 3.5, 0.1, true);
    EXPECT_EQ(pump.target_rpm(), 0);
    EXPECT_GT(pump.actual_rpm(), 0.0);
    EXPECT_LT(pump.actual_rpm(), 3000.0);
}

TEST(CAN, PumpIgnoresCommandsAboveCalibration) {
    eae::SimulatedCanBus bus;
    eae::SimulatedPump pump(4600, 0.5);
    bus.send_to_pump(eae::encode_pump_command(6000));
    pump.update(bus, 0.0, 0.1, true);
    EXPECT_EQ(pump.target_rpm(), 0);
    EXPECT_DOUBLE_EQ(pump.actual_rpm(), 0.0);
}

TEST(CAN, PumpReportsDeviceFaultWithoutLosingCommunication) {
    eae::SimulatedCanBus bus;
    eae::SimulatedPump pump(4600, 0.5);
    bus.send_to_pump(eae::encode_pump_command(3000));
    pump.update(bus, 0.0, 0.1, true, true);
    const auto frame = bus.receive_at_controller();
    ASSERT_TRUE(frame);
    const auto decoded = eae::decode_pump_feedback(*frame);
    ASSERT_TRUE(decoded);
    EXPECT_TRUE(decoded->device_fault);
    EXPECT_EQ(decoded->rpm, 0);
}

TEST(CAN, IgnitionOffClearsPreviousPumpCommand) {
    eae::SimulatedCanBus bus;
    eae::SimulatedPump pump(4600, 0.5);
    bus.send_to_pump(eae::encode_pump_command(3000));
    pump.update(bus, 0.0, 0.1, true);
    pump.update(bus, 0.1, 0.1, false);
    pump.update(bus, 0.2, 0.1, true);
    EXPECT_EQ(pump.target_rpm(), 0);
    EXPECT_DOUBLE_EQ(pump.actual_rpm(), 0.0);
}

}  // namespace
