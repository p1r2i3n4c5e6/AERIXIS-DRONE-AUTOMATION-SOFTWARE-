// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file as2_behavior_tree_node.hpp
 *
 * ROS2 entrypoint for launching a node to run a behavior tree
 *
 * @authors Pedro Arias Pérez
 *          Rafael Perez-Segui
 *          Miguel Fernández Cortizas
 */


// Groot connection
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "as2_behavior_tree/execution.hpp"
#include "as2_behavior_tree/rescue_nodes.hpp"

#include "rclcpp/rclcpp.hpp"
#include "as2_behavior_tree/action/arm_service.hpp"
#include "as2_behavior_tree/action/echo.hpp"
#include "as2_behavior_tree/action/follow_path.hpp"
#include "as2_behavior_tree/action/get_origin.hpp"
#include "as2_behavior_tree/action/go_to_action.hpp"
#include "as2_behavior_tree/action/go_to_gps_action.hpp"
#include "as2_behavior_tree/action/gps_to_cartesian.hpp"
#include "as2_behavior_tree/action/land_action.hpp"
#include "as2_behavior_tree/action/offboard_service.hpp"
#include "as2_behavior_tree/action/send_event.hpp"
#include "as2_behavior_tree/action/set_origin.hpp"
#include "as2_behavior_tree/action/takeoff_action.hpp"
#include "as2_behavior_tree/condition/is_flying_condition.hpp"
#include "as2_behavior_tree/condition/is_person_detected.hpp"
#include "as2_behavior_tree/decorator/wait_for_alert.hpp"
#include "as2_behavior_tree/decorator/wait_for_event.hpp"


namespace
{
volatile std::sig_atomic_t interrupted = 0;
void requestShutdown(int) {interrupted = 1;}
}  // namespace

int main(int argc, char * argv[])
{
  // Keep ROS alive until haltTree has had a chance to cancel active actions.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestShutdown);
  std::signal(SIGTERM, requestShutdown);
  auto node = std::make_shared<rclcpp::Node>("bt_manager");

  node->declare_parameter<std::string>("tree", "");
  node->declare_parameter<std::string>("mission_id", "local");
  const auto run_id = std::to_string(
    std::chrono::system_clock::now().time_since_epoch().count());
  auto events = node->create_publisher<std_msgs::msg::String>(
    "mission/events", rclcpp::QoS(100).reliable().transient_local());
  std::mutex event_mutex;
  unsigned long event_sequence = 0;
  auto emit = [&](nlohmann::json event) {
      std::lock_guard<std::mutex> lock(event_mutex);
      event["schema_version"] = 1;
      event["mission_id"] = node->get_parameter("mission_id").as_string();
      event["run_id"] = run_id;
      event["drone_namespace"] = node->get_namespace();
      event["sequence"] = ++event_sequence;
      std_msgs::msg::String message;
      message.data = event.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
      events->publish(message);
      std::cout << "AS2_BT_EVENT " << message.data << std::endl;
    };
  bool cancel_requested = false;
  auto cancel_service = node->create_service<std_srvs::srv::Trigger>(
    "mission/cancel",
    [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      cancel_requested = true;
      response->success = true;
      response->message = "Cancellation requested; await terminal event. Physical hold not verified.";
    });
  as2_behavior_tree::ExecutionResult execution{"ERROR", 3, "Initialization failed"};
  try {
    node->declare_parameter<bool>("use_groot", false);
    node->declare_parameter<bool>("describe_nodes", false);
    node->declare_parameter<int>("groot_client_port", 1666);
    node->declare_parameter<int>("groot_server_port", 1667);
    node->declare_parameter<int>("server_timeout", 10000);  // miliseconds
    node->declare_parameter<int>("bt_loop_duration", 10);  // miliseconds
    node->declare_parameter<int>("wait_for_service_timeout", 5000);  // miliseconds
    std::string tree_description = node->get_parameter("tree").as_string();
    bool groot_logger = node->get_parameter("use_groot").as_bool();
    int groot_client_port = node->get_parameter("groot_client_port").as_int();
    int server_timeout = node->get_parameter("server_timeout").as_int();
    int bt_loop_duration = node->get_parameter("bt_loop_duration").as_int();
    int wait_for_service_timeout = node->get_parameter("wait_for_service_timeout").as_int();

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<as2_behavior_tree::ArmService>("Arm");
    factory.registerNodeType<as2_behavior_tree::DisarmService>("Disarm");
    factory.registerNodeType<as2_behavior_tree::OffboardService>("Offboard");
    factory.registerNodeType<as2_behavior_tree::TakeoffAction>("TakeOff");
    factory.registerNodeType<as2_behavior_tree::GoToAction>("GoTo");
    factory.registerNodeType<as2_behavior_tree::LandAction>("Land");
    factory.registerNodeType<as2_behavior_tree::IsFlyingCondition>("IsFlying");
    factory.registerNodeType<as2_behavior_tree::IsPersonDetected>("IsPersonDetected");
    factory.registerNodeType<as2_behavior_tree::WaitForEvent>("WaitForEvent");
    factory.registerNodeType<as2_behavior_tree::WaitForAlert>("WaitForAlert");
    factory.registerNodeType<as2_behavior_tree::SendEvent>("SendEvent");
    factory.registerNodeType<as2_behavior_tree::Echo>("Echo");
    factory.registerNodeType<as2_behavior_tree::SetOrigin>("SetOrigin");
    factory.registerNodeType<as2_behavior_tree::GetOrigin>("GetOrigin");
    factory.registerNodeType<as2_behavior_tree::GpsToCartesian>("GpsToCartesian");
    factory.registerNodeType<as2_behavior_tree::GoToGpsAction>("GoToGps");
    factory.registerNodeType<as2_behavior_tree::FollowPathAction>("FollowPath");

    auto rescue_context = std::make_shared<as2_behavior_tree::RescueContext>(
      node, node->get_parameter("mission_id").as_string(), run_id);
    as2_behavior_tree::registerRescueNodes(factory, rescue_context);

    if (node->get_parameter("describe_nodes").as_bool()) {
      nlohmann::json registry = nlohmann::json::object();
      for (const auto & item : factory.manifests()) {
        nlohmann::json ports = nlohmann::json::object();
        for (const auto & port : item.second.ports) {
          ports[port.first] = {{"type", port.second.typeName()},
            {"direction", static_cast<int>(port.second.direction())},
            {"has_default", !port.second.defaultValue().empty()},
            {"default", port.second.defaultValueString()},
            {"description", port.second.description()}};
        }
        registry[item.first] = {{"category", BT::toStr(item.second.type)}, {"ports", ports}};
      }
      std::cout << "AS2_BT_REGISTRY " << registry.dump() << std::endl;
      rclcpp::shutdown();
      return 0;
    }

    if (bt_loop_duration <= 0 || server_timeout <= 0 || wait_for_service_timeout <= 0) {
      throw std::invalid_argument("BT loop and service timeouts must be positive");
    }
    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", node);
    blackboard->set<std::chrono::milliseconds>(
      "server_timeout", std::chrono::milliseconds(server_timeout));
    blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration", std::chrono::milliseconds(bt_loop_duration));
    blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout", std::chrono::milliseconds(wait_for_service_timeout));
    auto tree = factory.createTreeFromFile(tree_description, blackboard);
    BT::StdCoutLogger logger_cout(tree);
    as2_behavior_tree::ExecutionLogger status_logger(tree, emit);
    std::shared_ptr<BT::Groot2Publisher> groot_pub;
    if (groot_logger) {
      groot_pub = std::make_shared<BT::Groot2Publisher>(tree, groot_client_port);
    }
    // Publish the complete instance list before the first tick, including unticked branches.
    nlohmann::json nodes = nlohmann::json::array();
    for (const auto & subtree : tree.subtrees) {
      for (const auto & entry : subtree->nodes) {
        std::vector<uint16_t> children;
        if (auto control = dynamic_cast<BT::ControlNode *>(entry.get())) {
          for (const auto * child : control->children()) {
            children.push_back(child->UID());
          }
        } else if (auto decorator = dynamic_cast<BT::DecoratorNode *>(entry.get())) {
          if (decorator->child()) {children.push_back(decorator->child()->UID());}
        }
        nodes.push_back(
          {{"node_uid", entry->UID()}, {"node_path", entry->fullPath()},
            {"node_name", entry->name()}, {"node_type", entry->registrationName()},
            {"node_category", BT::toStr(entry->type())}, {"children_uids", children},
            {"status", "IDLE"}});
      }
    }
    emit({{"event", "started"}, {"nodes", nodes}});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::WallRate loop_rate{std::chrono::milliseconds(bt_loop_duration)};
    execution = as2_behavior_tree::executeTree(
      tree, [&]() {return rclcpp::ok() && !interrupted && !cancel_requested;},
      [&]() {executor.spin_some();}, [&]() {loop_rate.sleep();});
  } catch (const std::exception & error) {
    execution = {"ERROR", 3, error.what()};
  }
  emit(
    {{"event", "finished"}, {"outcome", execution.outcome},
      {"exit_code", execution.exit_code}, {"detail", execution.detail},
      {"physical_hold_verified", false}});
  rclcpp::shutdown();
  return execution.exit_code;
}
