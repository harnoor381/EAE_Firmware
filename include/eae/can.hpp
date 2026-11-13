#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>

namespace eae {

// These IDs and payloads belong to this demo, not the WP32 production protocol.
constexpr std::uint32_t pump_command_id = 0x200;
constexpr std::uint32_t pump_feedback_id = 0x201;

struct CanFrame {
    std::uint32_t id = 0;
    std::uint8_t size = 0;
    std::array<std::uint8_t, 8> data{};
};

struct PumpFeedback {
    int rpm = 0;
    bool device_fault = false;
};

CanFrame encode_pump_command(int rpm);
std::optional<int> decode_pump_command(const CanFrame& frame);
CanFrame encode_pump_feedback(PumpFeedback feedback);
std::optional<PumpFeedback> decode_pump_feedback(const CanFrame& frame);
std::string describe(const CanFrame& frame);

enum class Direction { ToPump, ToController };

struct BusStatistics {
    std::size_t commands_sent = 0;
    std::size_t feedback_sent = 0;
    std::size_t commands_dropped = 0;
    std::size_t feedback_dropped = 0;
};

class SimulatedCanBus {
public:
    using Observer = std::function<void(Direction, const CanFrame&, bool)>;
    explicit SimulatedCanBus(std::size_t capacity = 64);
    void set_observer(Observer observer);
    void set_drop_messages(bool commands, bool feedback);
    bool send_to_pump(const CanFrame& frame);
    bool send_to_controller(const CanFrame& frame);
    std::optional<CanFrame> receive_at_pump();
    std::optional<CanFrame> receive_at_controller();
    const BusStatistics& statistics() const { return statistics_; }

private:
    bool send(Direction direction, const CanFrame& frame);
    std::optional<CanFrame> receive(std::deque<CanFrame>& queue);
    std::size_t capacity_;
    bool drop_commands_ = false;
    bool drop_feedback_ = false;
    std::deque<CanFrame> to_pump_;
    std::deque<CanFrame> to_controller_;
    BusStatistics statistics_;
    Observer observer_;
};

class PumpFeedbackMonitor {
public:
    explicit PumpFeedbackMonitor(double timeout_seconds);
    bool receive(const CanFrame& frame, double now_seconds);
    bool communication_ok(double now_seconds) const;
    void reset();
    PumpFeedback feedback() const { return feedback_; }

private:
    double timeout_seconds_;
    std::optional<double> last_valid_message_;
    PumpFeedback feedback_;
};

class SimulatedPump {
public:
    SimulatedPump(int maximum_rpm, double command_timeout_seconds);
    // Receive queued commands, update motor speed, and send status every 100 ms.
    void update(SimulatedCanBus& bus, double now_seconds, double dt_seconds,
                bool powered, bool device_fault = false);
    double actual_rpm() const { return actual_rpm_; }
    int target_rpm() const { return target_rpm_; }

private:
    int maximum_rpm_;
    double command_timeout_seconds_;
    int target_rpm_ = 0;
    double actual_rpm_ = 0.0;
    std::optional<double> last_valid_command_;
    std::optional<double> last_feedback_;
};

}  // namespace eae
