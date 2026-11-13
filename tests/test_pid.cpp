#include "eae/pid.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

namespace {

TEST(PID, HotterCoolantRequestsMoreCooling) {
    eae::Pid pid({2.0, 0.0, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(pid.update(75.0, 80.0, 0.1), 10.0);
    EXPECT_DOUBLE_EQ(pid.update(75.0, 85.0, 0.1), 20.0);
    EXPECT_DOUBLE_EQ(pid.update(75.0, 70.0, 0.1), 0.0);
}

TEST(PID, IntegralUsesElapsedTime) {
    eae::Pid pid({0.0, 1.0, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(pid.update(75.0, 85.0, 0.1), 1.0);
    EXPECT_DOUBLE_EQ(pid.update(75.0, 85.0, 0.2), 3.0);
}

TEST(PID, UpperOutputLimitDoesNotBuildUnboundedIntegral) {
    eae::Pid pid({10.0, 0.2, 0.0, 0.0});
    for (int i = 0; i < 10000; ++i) {
        EXPECT_DOUBLE_EQ(pid.update(75.0, 100.0, 0.1), 100.0);
    }
    EXPECT_DOUBLE_EQ(pid.integral_term(), 0.0);
    EXPECT_LT(pid.update(75.0, 76.0, 0.1), 11.0);
}

TEST(PID, LowerOutputLimitDoesNotBuildNegativeIntegral) {
    eae::Pid pid({10.0, 0.2, 0.0, 0.0});
    for (int i = 0; i < 1000; ++i) pid.update(75.0, 25.0, 0.1);
    EXPECT_DOUBLE_EQ(pid.integral_term(), 0.0);
    EXPECT_GT(pid.update(75.0, 76.0, 0.1), 10.0);
}

TEST(PID, LargeIntegralOnlyStepCanReachBothOutputLimits) {
    eae::Pid pid({0.0, 100.0, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(pid.update(75.0, 85.0, 1.0), 100.0);
    EXPECT_DOUBLE_EQ(pid.integral_term(), 100.0);
    EXPECT_DOUBLE_EQ(pid.update(75.0, 65.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(pid.integral_term(), 0.0);
}

TEST(PID, ArithmeticOverflowIsReportedInsteadOfReturningNaN) {
    eae::Pid pid({1e308, 0.0, 0.0, 0.0});
    EXPECT_THROW(pid.update(75.0, 85.0, 0.1), std::overflow_error);
    eae::Pid derivative_pid({0.0, 0.0, 1.0, 0.0});
    derivative_pid.update(0.0, -1e308, 0.1);
    EXPECT_THROW(derivative_pid.update(0.0, 1e308, 0.1), std::overflow_error);
}

TEST(PID, SetpointChangeDoesNotCauseDerivativeKick) {
    eae::Pid pid({0.0, 0.0, 2.0, 0.0});
    EXPECT_DOUBLE_EQ(pid.update(60.0, 75.0, 0.1), 0.0);
    EXPECT_DOUBLE_EQ(pid.update(70.0, 75.0, 0.1), 0.0);
    EXPECT_DOUBLE_EQ(pid.update(70.0, 76.0, 0.1), 20.0);
}

TEST(PID, DerivativeFilterReducesSuddenMeasurementChange) {
    eae::Pid pid({0.0, 0.0, 2.0, 0.5});
    pid.update(75.0, 75.0, 0.1);
    EXPECT_NEAR(pid.update(75.0, 76.0, 0.1), 20.0 / 6.0, 1e-10);
}

TEST(PID, ResetClearsIntegralAndMeasurementHistory) {
    eae::Pid pid({0.0, 1.0, 2.0, 0.0});
    pid.update(75.0, 80.0, 0.1);
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.integral_term(), 0.0);
    EXPECT_DOUBLE_EQ(pid.update(75.0, 75.0, 0.1), 0.0);
}

TEST(PID, InvalidArgumentsAreRejected) {
    EXPECT_THROW(eae::Pid({-1.0, 0.0, 0.0, 0.0}), std::invalid_argument);
    EXPECT_THROW(eae::Pid({0.0, 0.0, 0.0, -1.0}), std::invalid_argument);
    eae::Pid pid;
    EXPECT_THROW(pid.update(75.0, 80.0, 0.0), std::invalid_argument);
    EXPECT_THROW(pid.update(75.0, 80.0, -0.1), std::invalid_argument);
    EXPECT_THROW(pid.update(75.0, std::numeric_limits<double>::quiet_NaN(), 0.1),
                 std::invalid_argument);
}

}  // namespace
