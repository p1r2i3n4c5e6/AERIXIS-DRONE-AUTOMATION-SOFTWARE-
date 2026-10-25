// Copyright 2026 AS2 project contributors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef AS2_BEHAVIOR_TREE__CONDITION__IS_PERSON_DETECTED_HPP_
#define AS2_BEHAVIOR_TREE__CONDITION__IS_PERSON_DETECTED_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace as2_behavior_tree
{
// Observation-only condition. Success does not authorize approach or payload release.
class IsPersonDetected : public BT::ConditionNode
{
public:
  IsPersonDetected(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config),
    node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
  {
    std::string topic;
    if (!getInput("topic", topic) || topic.empty()) {
      throw BT::RuntimeError("IsPersonDetected requires a nonempty topic");
    }
    subscription_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
      topic, rclcpp::SensorDataQoS(),
      [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = message;
        received_ = std::chrono::steady_clock::now();
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "perception/detections", "Namespaced detections"),
      BT::InputPort<double>("max_age_s", 0.5, "Maximum image and reception age"),
      BT::InputPort<double>("min_confidence", 0.35, "Minimum person confidence")};
  }

  BT::NodeStatus tick() override
  {
    double max_age = 0.0;
    double confidence = 0.0;
    if (!getInput("max_age_s", max_age) || !getInput("min_confidence", confidence) ||
      !std::isfinite(max_age) || max_age <= 0.0 ||
      !std::isfinite(confidence) || confidence <= 0.0 || confidence > 1.0)
    {
      return BT::NodeStatus::FAILURE;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_ || latest_->header.frame_id.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    const double received_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - received_).count();
    const double image_age =
      (node_->now() -
      rclcpp::Time(latest_->header.stamp, node_->get_clock()->get_clock_type())).seconds();
    if (received_age > max_age || image_age < 0.0 || image_age > max_age) {
      return BT::NodeStatus::FAILURE;
    }
    for (const auto & detection : latest_->detections) {
      for (const auto & result : detection.results) {
        if (result.hypothesis.class_id == "person" &&
          std::isfinite(result.hypothesis.score) &&
          result.hypothesis.score >= confidence && result.hypothesis.score <= 1.0)
        {
          return BT::NodeStatus::SUCCESS;
        }
      }
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  std::mutex mutex_;
  vision_msgs::msg::Detection2DArray::ConstSharedPtr latest_;
  std::chrono::steady_clock::time_point received_;
};
}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__CONDITION__IS_PERSON_DETECTED_HPP_
