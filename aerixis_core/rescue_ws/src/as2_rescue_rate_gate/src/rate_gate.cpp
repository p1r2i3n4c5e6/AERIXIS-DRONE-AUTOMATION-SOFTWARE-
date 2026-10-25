// Rate gate: forwards high-rate topics at a lower rate for the Python rescue nodes.
//
// Why: ROS 2 Humble's Python executor rebuilds its wait set for every message it takes, a few
// milliseconds on a Raspberry Pi 5. With the real PX4 1.16 FC over Ethernet, pose / twist / IMU /
// tf arrive at ~100 Hz, and every Python node that listened to them spent 20-50 % of a core on
// that overhead alone (Pi 5, 2026-10-25: load 9 on 4 cores). This node takes them once, in C++,
// without decoding them, and republishes each at the rate the Python side actually needs.
//
// Parameters
//   topics  string[]  "in_topic out_topic type hz" per entry, e.g.
//                     "self_localization/pose rescue/in/pose geometry_msgs/msg/PoseStamped 25"
//   tf_in   string    tf topic to read (default /tf)
//   tf_out  string    gated tf output; empty = no tf gate
//   tf_hz   double    per child frame: every frame keeps its own rate, so a slow transform
//                     (gimbal, camera) is never dropped because a fast one (odometry) came first.
//
// The output is RELIABLE so both reliable and best-effort subscribers match.
#include <chrono>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{
using Clock = std::chrono::steady_clock;

// Forward when at least 90 % of the period has passed: input jitter must not halve the rate.
bool due(Clock::time_point & last, double period_s, Clock::time_point now)
{
  if (now - last >= std::chrono::duration<double>(0.9 * period_s)) {
    last = now;
    return true;
  }
  return false;
}

struct Gate
{
  rclcpp::GenericSubscription::SharedPtr sub;
  rclcpp::GenericPublisher::SharedPtr pub;
  double period_s{0.04};
  Clock::time_point last{};
};
}  // namespace

class RateGate : public rclcpp::Node
{
public:
  RateGate()
  : Node("rate_gate")
  {
    const auto entries = declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
    const auto tf_in = declare_parameter<std::string>("tf_in", "/tf");
    const auto tf_out = declare_parameter<std::string>("tf_out", "");
    const double tf_hz = declare_parameter<double>("tf_hz", 30.0);

    const auto in_qos = rclcpp::SensorDataQoS();                  // matches any publisher
    const auto out_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    for (const auto & entry : entries) {
      std::istringstream fields(entry);
      std::string in, out, type;
      double hz = 0.0;
      if (!(fields >> in >> out >> type >> hz) || hz <= 0.0) {
        throw std::invalid_argument("rate_gate: bad topics entry '" + entry +
                "' (expected 'in_topic out_topic type hz')");
      }
      auto gate = std::make_shared<Gate>();
      gate->period_s = 1.0 / hz;
      gate->pub = create_generic_publisher(out, type, out_qos);
      std::weak_ptr<Gate> weak = gate;
      gate->sub = create_generic_subscription(
        in, type, in_qos,
        [weak](std::shared_ptr<rclcpp::SerializedMessage> message) {
          auto g = weak.lock();
          if (g && due(g->last, g->period_s, Clock::now())) {
            g->pub->publish(*message);
          }
        });
      gates_.push_back(gate);
      RCLCPP_INFO(get_logger(), "%s -> %s at %.0f Hz", sub_name(gate->sub).c_str(),
                  gate->pub->get_topic_name(), hz);
    }

    if (!tf_out.empty()) {
      if (tf_hz <= 0.0) {
        throw std::invalid_argument("rate_gate: tf_hz must be > 0");
      }
      tf_period_s_ = 1.0 / tf_hz;
      // tf2_ros broadcasters and listeners are RELIABLE, depth 100.
      const auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
      tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(tf_out, tf_qos);
      tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        tf_in, tf_qos, [this](tf2_msgs::msg::TFMessage::UniquePtr message) {on_tf(std::move(message));});
      RCLCPP_INFO(get_logger(), "%s -> %s at %.0f Hz per frame", tf_sub_->get_topic_name(),
                  tf_pub_->get_topic_name(), tf_hz);
    }
  }

private:
  static std::string sub_name(const rclcpp::GenericSubscription::SharedPtr & sub)
  {
    return sub->get_topic_name();
  }

  void on_tf(tf2_msgs::msg::TFMessage::UniquePtr message)
  {
    const auto now = Clock::now();
    auto out = std::make_unique<tf2_msgs::msg::TFMessage>();
    for (auto & transform : message->transforms) {
      auto & last = tf_last_[transform.header.frame_id + '>' + transform.child_frame_id];
      if (due(last, tf_period_s_, now)) {
        out->transforms.push_back(std::move(transform));
      }
    }
    if (!out->transforms.empty()) {
      tf_pub_->publish(std::move(out));
    }
  }

  std::vector<std::shared_ptr<Gate>> gates_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  std::map<std::string, Clock::time_point> tf_last_;
  double tf_period_s_{1.0 / 30.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RateGate>());
  rclcpp::shutdown();
  return 0;
}
