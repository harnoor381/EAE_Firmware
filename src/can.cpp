#include "eae/can.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace eae {
namespace {

void encode_rpm(CanFrame& frame, int rpm) {
    if (rpm < 0 || rpm > 65535) {
        throw std::invalid_argument("CAN RPM must fit in an unsigned 16-bit value");
    }
    // The least significant byte is sent first.
    frame.data[0] = static_cast<std::uint8_t>(rpm & 0xFF);
    frame.data[1] = static_cast<std::uint8_t>((rpm >> 8) & 0xFF);
}

int decode_rpm(const CanFrame& frame) {
    return static_cast<int>(frame.data[0]) |
        (static_cast<int>(frame.data[1]) << 8);
}

}  // namespace

CanFrame encode_pump_command(int rpm) {
    CanFrame frame;
    frame.id = pump_command_id;
    frame.size = 2;
    encode_rpm(frame, rpm);
    return frame;
}

std::optional<int> decode_pump_command(const CanFrame& frame) {
    if (frame.id != pump_command_id || frame.size != 2) return std::nullopt;
    return decode_rpm(frame);
}

CanFrame encode_pump_feedback(PumpFeedback feedback) {
    CanFrame frame;
    frame.id = pump_feedback_id;
    frame.size = 3;
    encode_rpm(frame, feedback.rpm);
    frame.data[2] = feedback.device_fault ? 1U : 0U;
    return frame;
}

std::optional<PumpFeedback> decode_pump_feedback(const CanFrame& frame) {
    if (frame.id != pump_feedback_id || frame.size != 3 || frame.data[2] > 1U) {
        return std::nullopt;
    }
    return PumpFeedback{decode_rpm(frame), frame.data[2] == 1U};
}

std::string describe(const CanFrame& frame) {
    std::ostringstream out;
    out << "id=0x" << std::hex << std::uppercase << std::setfill('0')
        << std::setw(3) << frame.id << std::dec
        << " dlc=" << static_cast<int>(frame.size) << " data=";
    const auto count = std::min(static_cast<std::size_t>(frame.size), frame.data.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (i != 0) out << ' ';
        out << std::hex << std::setw(2) << static_cast<int>(frame.data[i]);
    }
    return out.str();
}

SimulatedCanBus::SimulatedCanBus(std::size_t capacity) : capacity_(capacity) {
    if (capacity == 0) throw std::invalid_argument("CAN queue capacity must be positive");
}

void SimulatedCanBus::set_observer(Observer observer) {
    observer_ = std::move(observer);
}

void SimulatedCanBus::set_drop_messages(bool commands, bool feedback) {
    drop_commands_ = commands;
    drop_feedback_ = feedback;
}

bool SimulatedCanBus::send_to_pump(const CanFrame& frame) {
    return send(Direction::ToPump, frame);
}

bool SimulatedCanBus::send_to_controller(const CanFrame& frame) {
    return send(Direction::ToController, frame);
}

bool SimulatedCanBus::send(Direction direction, const CanFrame& frame) {
    const bool command = direction == Direction::ToPump;
    auto& queue = command ? to_pump_ : to_controller_;
    const bool drop = (command ? drop_commands_ : drop_feedback_) ||
                      queue.size() >= capacity_;
    if (command) {
        ++statistics_.commands_sent;
        if (drop) ++statistics_.commands_dropped;
    } else {
        ++statistics_.feedback_sent;
        if (drop) ++statistics_.feedback_dropped;
    }
    if (!drop) queue.push_back(frame);
    if (observer_) observer_(direction, frame, !drop);
    return !drop;
}

std::optional<CanFrame> SimulatedCanBus::receive(std::deque<CanFrame>& queue) {
    if (queue.empty()) return std::nullopt;
    const CanFrame frame = queue.front();
    queue.pop_front();
    return frame;
}

std::optional<CanFrame> SimulatedCanBus::receive_at_pump() {
    return receive(to_pump_);
}

std::optional<CanFrame> SimulatedCanBus::receive_at_controller() {
    return receive(to_controller_);
}

PumpFeedbackMonitor::PumpFeedbackMonitor(double timeout_seconds)
    : timeout_seconds_(timeout_seconds) {
    if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0) {
        throw std::invalid_argument("Feedback timeout must be positive and finite");
    }
}

bool PumpFeedbackMonitor::receive(const CanFrame& frame, double now_seconds) {
    const auto decoded = decode_pump_feedback(frame);
    if (!decoded || !std::isfinite(now_seconds) || now_seconds < 0.0 ||
        (last_valid_message_ && now_seconds < *last_valid_message_)) {
        return false;
    }
    feedback_ = *decoded;
    last_valid_message_ = now_seconds;
    return true;
}

bool PumpFeedbackMonitor::communication_ok(double now_seconds) const {
    return last_valid_message_ && std::isfinite(now_seconds) &&
        now_seconds >= *last_valid_message_ &&
        now_seconds - *last_valid_message_ <= timeout_seconds_ + 1e-12;
}

void PumpFeedbackMonitor::reset() {
    last_valid_message_.reset();
    feedback_ = {};
}

SimulatedPump::SimulatedPump(int maximum_rpm, double command_timeout_seconds)
    : maximum_rpm_(maximum_rpm), command_timeout_seconds_(command_timeout_seconds) {
    if (maximum_rpm <= 0 || maximum_rpm > 65535 ||
        !std::isfinite(command_timeout_seconds) || command_timeout_seconds <= 0.0) {
        throw std::invalid_argument("Invalid simulated pump settings");
    }
}

void SimulatedPump::update(SimulatedCanBus& bus, double now_seconds,
                           double dt_seconds, bool powered, bool device_fault) {
    if (!std::isfinite(now_seconds) || now_seconds < 0.0 ||
        !std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
        throw std::invalid_argument("Pump time and time step are invalid");
    }

    // A powered-off pump ignores commands. An ignition restart must receive
    // a new command; it cannot reuse a command from the previous run.
    while (const auto frame = bus.receive_at_pump()) {
        const auto rpm = decode_pump_command(*frame);
        if (powered && rpm && *rpm <= maximum_rpm_) {
            target_rpm_ = *rpm;
            last_valid_command_ = now_seconds;
        }
    }
    if (!powered) {
        target_rpm_ = 0;
        actual_rpm_ = 0.0;
        last_valid_command_.reset();
        last_feedback_.reset();
        return;
    }

    // Model a local pump watchdog. Its stop-on-timeout setting is a demo
    // assumption and must be checked against the real WP32 calibration.
    if (!last_valid_command_ ||
        now_seconds - *last_valid_command_ > command_timeout_seconds_ + 1e-12 ||
        device_fault) {
        target_rpm_ = 0;
    }

    // Motor speed changes gradually, so requested RPM and actual RPM differ.
    const double alpha = 1.0 - std::exp(-dt_seconds / 0.4);
    actual_rpm_ += alpha * (static_cast<double>(target_rpm_) - actual_rpm_);
    if (device_fault) actual_rpm_ = 0.0;

    // Feedback continues even while stopped or reporting a device fault.
    if (!last_feedback_ || now_seconds - *last_feedback_ >= 0.1 - 1e-12) {
        bus.send_to_controller(encode_pump_feedback(
            {static_cast<int>(std::lround(actual_rpm_)), device_fault}));
        last_feedback_ = now_seconds;
    }
}

}  // namespace eae
