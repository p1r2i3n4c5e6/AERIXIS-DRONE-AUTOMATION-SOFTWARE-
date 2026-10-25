// Copyright 2026 AS2 project contributors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef AS2_BEHAVIOR_TREE__EXECUTION_HPP_
#define AS2_BEHAVIOR_TREE__EXECUTION_HPP_

#include <functional>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/contrib/json.hpp"
#include "behaviortree_cpp/loggers/abstract_logger.h"

namespace as2_behavior_tree
{

struct ExecutionResult
{
  std::string outcome;
  int exit_code;
  std::string detail;
};

// pump() processes cancellation before each tick. idle() controls the tick rate.
// Cancellation means the tree was halted, not that physical motion was verified stopped.
inline ExecutionResult executeTree(
  BT::Tree & tree, const std::function<bool()> & keep_running,
  const std::function<void()> & pump, const std::function<void()> & idle)
{
  try {
    while (true) {
      pump();
      if (!keep_running()) {
        tree.haltTree();
        return {"CANCELLED", 2, "Tree halted; physical hold is not verified"};
      }
      const auto status = tree.tickExactlyOnce();
      if (status == BT::NodeStatus::SUCCESS) {
        return {"SUCCESS", 0, "Root returned SUCCESS"};
      }
      if (status == BT::NodeStatus::FAILURE) {
        tree.haltTree();
        return {"FAILURE", 1, "Root returned FAILURE"};
      }
      if (status != BT::NodeStatus::RUNNING) {
        tree.haltTree();
        return {"ERROR", 3, "Root did not return SUCCESS, FAILURE or RUNNING"};
      }
      idle();
    }
  } catch (const std::exception & error) {
    std::string detail = error.what();
    try {
      tree.haltTree();
    } catch (const std::exception & halt_error) {
      detail += std::string("; halt failed: ") + halt_error.what();
    }
    return {"ERROR", 3, detail};
  }
}

class ExecutionLogger : public BT::StatusChangeLogger
{
public:
  using Sink = std::function<void (nlohmann::json)>;

  ExecutionLogger(const BT::Tree & tree, Sink sink)
  : BT::StatusChangeLogger(tree.rootNode()), sink_(std::move(sink))
  {
    enableTransitionToIdle(true);
  }

  ~ExecutionLogger() override
  {
    unsubscribeFromTreeChanges();
  }

  void callback(
    BT::Duration, const BT::TreeNode & node,
    BT::NodeStatus previous, BT::NodeStatus status) override
  {
    sink_(
      {
        {"event", "node_status"}, {"node_uid", node.UID()},
        {"node_path", node.fullPath()}, {"node_name", node.name()},
        {"node_type", node.registrationName()},
        {"previous_status", BT::toStr(previous)}, {"status", BT::toStr(status)}});
  }

  void flush() override {}

private:
  Sink sink_;
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__EXECUTION_HPP_
