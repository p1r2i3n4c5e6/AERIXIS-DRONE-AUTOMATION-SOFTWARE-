// Copyright 2026 AS2 project contributors
// SPDX-License-Identifier: BSD-3-Clause

#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "as2_behavior_tree/execution.hpp"

namespace
{
BT::Tree makeTree(BT::BehaviorTreeFactory & factory, const std::string & body)
{
  return factory.createTreeFromText(
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">" + body +
    "</BehaviorTree></root>");
}

TEST(Execution, SuccessAndFailureHaveDifferentExitCodes)
{
  BT::BehaviorTreeFactory factory;
  for (const auto & item : std::vector<std::pair<std::string, int>>{
      {"AlwaysSuccess", 0}, {"AlwaysFailure", 1}})
  {
    auto tree = makeTree(factory, "<" + item.first + "/>");
    auto result = as2_behavior_tree::executeTree(tree, [] {return true;}, [] {}, [] {});
    EXPECT_EQ(result.exit_code, item.second);
    EXPECT_EQ(result.outcome, item.second == 0 ? "SUCCESS" : "FAILURE");
  }
}

TEST(Execution, CancellationPumpsBetweenTicksAndHaltsRunningNode)
{
  BT::BehaviorTreeFactory factory;
  auto tree = makeTree(factory, "<Sleep msec=\"10000\"/>");
  int pumps = 0;
  int idles = 0;
  auto result = as2_behavior_tree::executeTree(
    tree, [&] {return pumps < 2;}, [&] {++pumps;}, [&] {++idles;});
  EXPECT_EQ(result.outcome, "CANCELLED");
  EXPECT_EQ(result.exit_code, 2);
  EXPECT_EQ(pumps, 2);
  EXPECT_EQ(idles, 1);
  EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
}

TEST(Execution, CancelBeforeFirstTickDoesNotExecuteAction)
{
  BT::BehaviorTreeFactory factory;
  int calls = 0;
  factory.registerSimpleAction(
    "Work", [&](BT::TreeNode &) {
      ++calls;
      return BT::NodeStatus::SUCCESS;
    });
  auto tree = makeTree(factory, "<Work/>");
  const auto result = as2_behavior_tree::executeTree(
    tree, [] {return false;}, [] {}, [] {});
  EXPECT_EQ(result.outcome, "CANCELLED");
  EXPECT_EQ(calls, 0);
}

TEST(Execution, ExceptionIsErrorAndNotSuccess)
{
  BT::BehaviorTreeFactory factory;
  factory.registerSimpleAction(
    "Throw", [](BT::TreeNode &) -> BT::NodeStatus {
      throw std::runtime_error("deliberate test failure");
    });
  auto tree = makeTree(factory, "<Throw/>");
  const auto result = as2_behavior_tree::executeTree(tree, [] {return true;}, [] {}, [] {});
  EXPECT_EQ(result.outcome, "ERROR");
  EXPECT_EQ(result.exit_code, 3);
  EXPECT_NE(result.detail.find("deliberate test failure"), std::string::npos);
}

TEST(Execution, SkippedRootIsNotMissionSuccess)
{
  BT::BehaviorTreeFactory factory;
  auto tree = makeTree(factory, "<AlwaysSuccess _skipIf=\"true\"/>");
  const auto result = as2_behavior_tree::executeTree(tree, [] {return true;}, [] {}, [] {});
  EXPECT_EQ(result.outcome, "ERROR");
}

TEST(Execution, RepeatedNodeTypesHaveDistinctIdentityAndUntickedBranchStaysIdle)
{
  BT::BehaviorTreeFactory factory;
  auto tree = makeTree(
    factory, "<Sequence><AlwaysSuccess/><AlwaysSuccess/>"
    "<AlwaysFailure/><AlwaysSuccess/></Sequence>");
  std::vector<nlohmann::json> events;
  as2_behavior_tree::ExecutionLogger logger(
    tree, [&](nlohmann::json event) {events.push_back(event);});
  const auto result = as2_behavior_tree::executeTree(tree, [] {return true;}, [] {}, [] {});
  EXPECT_EQ(result.outcome, "FAILURE");
  std::vector<int> completed;
  for (const auto & event : events) {
    if (event["node_type"] == "AlwaysSuccess" && event["status"] == "SUCCESS") {
      completed.push_back(event["node_uid"].get<int>());
    }
  }
  ASSERT_EQ(completed.size(), 2u);
  EXPECT_NE(completed[0], completed[1]);
  EXPECT_EQ(tree.subtrees.front()->nodes.back()->status(), BT::NodeStatus::IDLE);
}
}  // namespace
