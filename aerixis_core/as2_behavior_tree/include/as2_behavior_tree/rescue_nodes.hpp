// Copyright 2026 AS2 project contributors
// SPDX-License-Identifier: BSD-3-Clause
#ifndef AS2_BEHAVIOR_TREE__RESCUE_NODES_HPP_
#define AS2_BEHAVIOR_TREE__RESCUE_NODES_HPP_
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace as2_behavior_tree
{
// Callbacks and BT ticks share the runner's single-threaded executor.
struct RescueContext
{
  using Clock = std::chrono::steady_clock;
  rclcpp::Node::SharedPtr node;
  std::string mission, run, boot;
  unsigned long counter = 0, sequence = 0;
  nlohmann::json state;
  Clock::time_point received;
  std::unordered_map<std::string, std::string> results;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub, result_sub;
  RescueContext(rclcpp::Node::SharedPtr n, std::string m, std::string r)
  : node(n), mission(m), run(r)
  {
    commands = node->create_publisher<std_msgs::msg::String>("rescue/command", 20);
    status_sub = node->create_subscription<std_msgs::msg::String>(
      "rescue/state", 10,
      [this](const std_msgs::msg::String & msg) {
        try {
          auto value = nlohmann::json::parse(msg.data);
          const double age = node->now().seconds() - value.at("stamp").get<double>();
          const auto b = value.at("boot_id").get<std::string>();
          const auto s = value.at("sequence").get<unsigned long>();
          if (value.at("schema_version") != 1 || age < 0 || age > 0.5 ||
          (b == boot && s <= sequence)) {return;}
          boot = b; sequence = s; state = value; received = Clock::now();
        } catch (const std::exception &) {state = nullptr;}
      });
    result_sub = node->create_subscription<std_msgs::msg::String>(
      "rescue/result", 100,
      [this](const std_msgs::msg::String & msg) {
        try {
          auto value = nlohmann::json::parse(msg.data);
          const auto id = value.at("command_id").get<std::string>();
          if (value.at("mission_id") == mission && value.at("run_id") == run &&
          results.count(id))
          {
            const auto result = value.at("status").get<std::string>();
            if (result == "SUCCESS" || result == "FAILURE") {results[id] = result;}
            if (result == "FAILURE") {
              RCLCPP_WARN(
                node->get_logger(), "Rescue operation failed: %s",
                value.value("reason", "unspecified").c_str());
            }
          }
        } catch (const std::exception &) {}
      });
  }
  bool flag(const std::string & name) const
  {
    if (state.is_null() || std::chrono::duration<double>(Clock::now() - received).count() > 0.5) {
      return false;
    }
    try {
      if (state.contains("mission_id") && !state["mission_id"].is_null() &&
        state["mission_id"] != "" && state["mission_id"] != mission) {return false;}
      return state.at("flags").at(name).get<bool>();
    } catch (...) {return false;}
  }
  void publish(nlohmann::json value)
  {
    value["schema_version"] = 1; value["mission_id"] = mission; value["run_id"] = run;
    value["stamp"] = node->now().seconds();
    std_msgs::msg::String message; message.data = value.dump(); commands->publish(message);
  }
};

class RescueAction : public BT::StatefulActionNode
{
public:
  RescueAction(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RescueContext> context, std::string operation)
  : BT::StatefulActionNode(name, config), context_(context), operation_(operation) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout_s", 30.0, "Bounded operation deadline"),
      BT::InputPort<std::string>("swarm_id", "", "Common mission group shared across aircraft"),
      BT::InputPort<double>("height", 2.0, "Takeoff height metres"),
      BT::InputPort<std::string>(
        "target_id", "",
        "Shared target/zone ID; never a pixel coordinate"),
      BT::InputPort<std::string>("segment_id", "", "Persistent coverage segment ID"),
      BT::InputPort<std::string>("pose", "", "Validated world-frame x;y;z in metres"),
      BT::InputPort<double>("speed", 0.5, "Maximum translation speed m/s"),
      BT::InputPort<double>("radius_m", 10.0, "OrbitTarget: circle radius around the person"),
      BT::InputPort<double>("laps", 1.0, "OrbitTarget: number of full circles"),
      BT::InputPort<double>("lost_timeout_s", 3.0, "Vision ops: fail after the person is unseen this long"),
      BT::InputPort<double>("duration_s", 0.0, "TrackTarget: succeed after tracking this long, 0 = until lost"),
      BT::InputPort<double>("suppress_s", 8.0, "ReleaseTargetLock: no new lock for this long"),
      BT::InputPort<double>("pitch_deg", -45.0, "SetGimbalSearch: gimbal pitch, -90 = straight down"),
      BT::InputPort<double>("settle_s", 1.5, "ApproachOverhead: overhead must hold this long"),
      BT::InputPort<std::string>("area", "", "SetSearchArea: plan-frame polygon x,y;x,y;... (metres)"),
      BT::InputPort<double>("yaw_rate_dps", 12.0, "ScanArea: slow yaw sweep rate while holding")};
  }
  BT::NodeStatus onStart() override
  {
    getInput("timeout_s", timeout_);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || timeout_ > 1800) {
      return BT::NodeStatus::FAILURE;
    }
    std::string target, segment, pose, swarm; double speed = 0, height = 2;
    getInput("swarm_id", swarm); getInput("target_id", target); getInput("segment_id", segment);
    getInput("pose", pose); getInput("speed", speed); getInput("height", height);
    double radius = 10, laps = 1, lost = 3, duration = 0, suppress = 8, pitch = -45, settle = 1.5;
    getInput("radius_m", radius); getInput("laps", laps); getInput("lost_timeout_s", lost);
    getInput("duration_s", duration); getInput("suppress_s", suppress);
    getInput("pitch_deg", pitch); getInput("settle_s", settle);
    std::string area; double yaw_rate = 12;
    getInput("area", area); getInput("yaw_rate_dps", yaw_rate);
    id_ = context_->run + ":" + std::to_string(UID()) + ":" + std::to_string(++context_->counter);
    context_->results[id_] = "RUNNING"; started_ = RescueContext::Clock::now();
    pending_ = nlohmann::json(
      {{"command_id", id_}, {"operation", operation_},
        {"timeout_s", timeout_}, {"swarm_id", swarm}, {"target_id", target}, {"segment_id",
          segment},
        {"pose", pose}, {"speed", speed}, {"height", height}, {"radius_m", radius},
        {"laps", laps}, {"lost_timeout_s", lost}, {"duration_s", duration},
        {"suppress_s", suppress}, {"pitch_deg", pitch}, {"settle_s", settle},
        {"area", area}, {"yaw_rate_dps", yaw_rate}});
    dispatched_ = false;
    if (context_->commands->get_subscription_count() > 0) {
      context_->publish(pending_); dispatched_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    if (!dispatched_ && context_->commands->get_subscription_count() > 0) {
      context_->publish(pending_); dispatched_ = true;
    }
    auto result = context_->results[id_];
    if (result == "SUCCESS" || result == "FAILURE") {
      context_->results.erase(id_);
      return result == "SUCCESS" ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    if (std::chrono::duration<double>(RescueContext::Clock::now() - started_).count() > timeout_) {
      onHalted(); return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override
  {
    if (dispatched_) {context_->publish({{"command_id", id_}, {"operation", "CancelOperation"}});}
    context_->results.erase(id_);
  }

private:
  std::shared_ptr<RescueContext> context_;
  std::string operation_, id_;
  nlohmann::json pending_;
  bool dispatched_ = false;
  double timeout_ = 30;
  RescueContext::Clock::time_point started_;
};

class RescueCondition : public BT::ConditionNode
{
public:
  RescueCondition(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RescueContext> context, std::string flag)
  : BT::ConditionNode(name, config), context_(context), flag_(flag) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {return context_->flag(flag_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;}

private:
  std::shared_ptr<RescueContext> context_;
  std::string flag_;
};

class StableFor : public BT::DecoratorNode
{
public:
  StableFor(const std::string & name, const BT::NodeConfig & config)
  : BT::DecoratorNode(name, config) {}
  static BT::PortsList providedPorts()
  {return {BT::InputPort<double>("seconds", 2.0, "Continuous child success required")};}
  BT::NodeStatus tick() override
  {
    double seconds = 0; getInput("seconds", seconds);
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 60) {return BT::NodeStatus::FAILURE;}
    const auto result = child_node_->executeTick();
    if (result != BT::NodeStatus::SUCCESS) {
      waiting_ = false;
      if (result != BT::NodeStatus::RUNNING) {resetChild();}
      return result;
    }
    resetChild();
    if (!waiting_) {since_ = RescueContext::Clock::now(); waiting_ = true;}
    return std::chrono::duration<double>(RescueContext::Clock::now() - since_).count() >= seconds ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void halt() override {waiting_ = false; BT::DecoratorNode::halt();}

private:
  bool waiting_ = false;
  RescueContext::Clock::time_point since_;
};

class SafetyGate : public BT::DecoratorNode
{
public:
  SafetyGate(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RescueContext> context)
  : BT::DecoratorNode(name, config), context_(context) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>(
        "flag", "VehicleHealthy",
        "Fresh supervisor flag required every tick")};
  }
  BT::NodeStatus tick() override
  {
    std::string flag; getInput("flag", flag);
    if (!context_->flag(flag)) {resetChild(); return BT::NodeStatus::FAILURE;}
    auto result = child_node_->executeTick();
    if (result != BT::NodeStatus::RUNNING) {resetChild();}
    return result;
  }

private:
  std::shared_ptr<RescueContext> context_;
};

class FailureLatch : public BT::DecoratorNode
{
public:
  FailureLatch(const std::string & name, const BT::NodeConfig & config)
  : BT::DecoratorNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    if (failed_) {return BT::NodeStatus::FAILURE;}
    auto result = child_node_->executeTick();
    if (result == BT::NodeStatus::FAILURE) {failed_ = true;}
    if (result != BT::NodeStatus::RUNNING) {resetChild();}
    return result;
  }

private:
  bool failed_ = false;  // Intentionally survives halt; clear only by a new tree instance.
};

// Child 0 is the task; child 1 is recovery. Recovery completion never turns a failed task green.
class RecoverThenFail : public BT::ControlNode
{
public:
  RecoverThenFail(const std::string & name, const BT::NodeConfig & config)
  : BT::ControlNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 2) {
      throw BT::RuntimeError("RecoverThenFail requires task and recovery");
    }
    if (!recovering_) {
      auto result = children_nodes_[0]->executeTick();
      if (result == BT::NodeStatus::RUNNING) {return result;}
      if (result == BT::NodeStatus::SUCCESS) {haltChildren(); return result;}
      haltChild(0); recovering_ = true;
    }
    if (children_nodes_[1]->executeTick() == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    haltChildren(); return BT::NodeStatus::FAILURE;
  }
  void halt() override {recovering_ = false; BT::ControlNode::halt();}

private:
  bool recovering_ = false;
};

inline void registerRescueNodes(
  BT::BehaviorTreeFactory & factory,
  const std::shared_ptr<RescueContext> & context)
{
  for (const std::string operation : {"CheckpointCoverage", "PauseCoverage", "ConfirmHold",
      "TrackPerson", "GeotagTarget", "ApproachDeliveryZone", "VerifyDeliveryZone", "ReleasePayload",
      "VerifyPayloadRelease", "ResumeCoverage", "ReportObservation", "RequestRecovery",
      "AcquireAssignment", "ReleaseAssignment", "SurveyWaypoint", "CompleteRescue",
      "ManagedTakeoff", "ManagedLand", "ReportPayloadOutcome",
      // A8 tracker (gimbal-guided) rescue operations
      "TrackTarget", "ApproachOverhead", "OrbitTarget", "GeotagOverhead", "DropPayload",
      "ReleaseTargetLock", "SetGimbalSearch", "ReturnToLaunch", "SetSearchArea", "ScanArea",
      // Mission 7 delivery drones (2026-11-01): claim a geotag heard over the mesh, fly, drop
      "AcquireTarget", "FlyToTarget", "DropAtTarget"})
  {
    factory.registerNodeType<RescueAction>(operation, context, operation);
  }
  for (const std::string flag : {"VehicleHealthy", "CameraFresh", "GimbalStable", "TrackConfirmed",
      "GeolocationValid", "HoldConfirmed", "DeliveryZoneClear", "PayloadLoaded", "SafeToRelease",
      "AbortRequested", "RescuePending", "AssignmentOwned", "PeersFresh", "SeparationSafe",
      "PayloadReleased", "CenteredOnTarget", "PayloadOperatorConfirmed",
      "HumanDetected", "TargetLocked", "OverheadTarget", "PayloadAvailable", "VisionHealthy",
      "FlightAvailable"})
  {
    factory.registerNodeType<RescueCondition>(flag, context, flag);
  }
  factory.registerNodeType<StableFor>("StableFor");
  factory.registerNodeType<SafetyGate>("SafetyGate", context);
  factory.registerNodeType<FailureLatch>("FailureLatch");
  factory.registerNodeType<RecoverThenFail>("RecoverThenFail");
}
}  // namespace as2_behavior_tree
#endif  // AS2_BEHAVIOR_TREE__RESCUE_NODES_HPP_
