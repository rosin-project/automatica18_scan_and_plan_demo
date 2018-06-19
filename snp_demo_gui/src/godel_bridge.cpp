// Copyright 2018 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "godel_bridge.h"

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include "godel_msgs/SurfaceDetection.h"
#include "godel_msgs/SelectSurface.h"

#include <actionlib/client/simple_action_client.h>
#include <godel_msgs/ProcessPlanningAction.h>

#include "godel_msgs/GetAvailableMotionPlans.h"
#include "godel_msgs/SelectMotionPlanAction.h"

namespace snp_demo_gui {

CommandResult makeError(const std::string &e) {
  ROS_ERROR_STREAM(e);
  return CommandResult(false,e);
}
CommandResult makeActionError(const actionlib::SimpleClientGoalState &state) {
  std::stringstream sstr;
  sstr << state.toString();
  std::string text = state.getText();
  if(!text.empty()) sstr << " - " << text;
  return makeError(sstr.str());
}

CommandResult run_scan(CommandLogFunc log) {
  godel_msgs::SurfaceDetection srv;
  srv.request.action = srv.request.SCAN_AND_FIND_ONLY;
  srv.request.use_default_parameters = true;
  if (!ros::service::call("surface_detection", srv)) return makeError("Unable to call surface detection service");
  else if (!srv.response.surfaces_found) return makeError("No surface found");

  godel_msgs::SelectSurface srv2;
  srv2.request.action = srv2.request.SELECT_ALL;
  if (!ros::service::call("select_surface", srv2)) return makeError("Unable to call surface selection service");
  return CommandResult(srv2.response.succeeded , srv2.response.succeeded ? "": "Could not select_surface");
}

class GodelBridgeImpl {
   ros::NodeHandle nh_;
   using ProcessPlanningActionClient = actionlib::SimpleActionClient<godel_msgs::ProcessPlanningAction>;
   ProcessPlanningActionClient process_planning_action_client_;
   using SelectMotionPlanActionClient = actionlib::SimpleActionClient<godel_msgs::SelectMotionPlanAction>;
   SelectMotionPlanActionClient select_motion_plan_action_client_;
   std::vector<std::string> plans_;
   moveit::planning_interface::MoveGroupInterface move_group_;
public:
  GodelBridgeImpl()
  : process_planning_action_client_("process_planning_as"),
    select_motion_plan_action_client_("select_motion_plan_as"),
    move_group_(nh_.param("/godel_process_planning/blend_group", std::string("manipulator_tcp")))
  {
  }

  CommandResult home(CommandLogFunc log) {
    move_group_.setPlannerId("PTP");
    move_group_.setNamedTarget("home");
    move_group_.setStartStateToCurrentState();
    moveit::planning_interface::MoveItErrorCode eCode = move_group_.move();
    if(!eCode) {
      std::stringstream sstr;
      sstr << eCode;
      return makeError(sstr.str());
    }
    return CommandResult(true, "");
  }

  CommandResult plan(CommandLogFunc log) {
    plans_.clear();
    godel_msgs::ProcessPlanningGoal goal;
    goal.action = goal.GENERATE_MOTION_PLAN_AND_PREVIEW;
    goal.use_default_parameters = true;
    process_planning_action_client_.sendGoal(
          goal, 0, 0,
          [log](const godel_msgs::ProcessPlanningFeedbackConstPtr& feedback) { log(feedback->last_completed); }
    );
    if(!process_planning_action_client_.waitForServer(ros::Duration(1.0))) return makeError("Cannot reach planning server");
    if(!process_planning_action_client_.waitForResult()) return makeError("Timeout while waiting for result");
    actionlib::SimpleClientGoalState state = process_planning_action_client_.getState();
    if(state != actionlib::SimpleClientGoalState::SUCCEEDED) return makeActionError(state);

    if(!process_planning_action_client_.getResult()->succeeded) return makeError(state.getText());

    godel_msgs::GetAvailableMotionPlans srv;
    if (!ros::service::call("get_available_motion_plans", srv)) return makeError("Unable to call get_available_motion_plans service");

    plans_ = srv.response.names;
    if(plans_.empty()) return makeError("No motion plans available");

    std::stringstream sstr;
    sstr << "Plan(s):";
    for(const std::string &s: plans_) sstr << " " << s;
    log(sstr.str());

    return CommandResult(true, "");
  }

  CommandResult runPlans(CommandLogFunc log, bool simulate) {
    if(plans_.empty()) return makeError("No motion plans available");

    godel_msgs::SelectMotionPlanGoal goal;
    goal.simulate = simulate;
    goal.wait_for_execution = true;

    for(const std::string &plan: plans_){
      log("Run " + plan);
      goal.name = plan;

      if(!select_motion_plan_action_client_.waitForServer(ros::Duration(1.0))) return makeError("Cannot reach motion server");

      actionlib::SimpleClientGoalState state = select_motion_plan_action_client_.sendGoalAndWait(goal);

      switch(select_motion_plan_action_client_.getResult()->code) {
      case  godel_msgs::SelectMotionPlanResult::NO_SUCH_NAME:
        return makeError("no such name");
      case  godel_msgs::SelectMotionPlanResult::TIMEOUT:
        return makeError("timed out");
      case  godel_msgs::SelectMotionPlanResult::SUCCESS:
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) continue;
        // else fall-through
      default:
        return makeActionError(state);
      }
    }
    return CommandResult(true, "");
  }
};

GodelBridge::GodelBridge()
: impl_(new GodelBridgeImpl())
{}

CommandResult GodelBridge::home(CommandLogFunc log) { return impl_->home(log); }
CommandResult GodelBridge::scan(CommandLogFunc log) { return run_scan(log); }
CommandResult GodelBridge::plan(CommandLogFunc log) { return impl_->plan(log); }
CommandResult GodelBridge::simulate(CommandLogFunc log) { return impl_->runPlans(log, true); }
CommandResult GodelBridge::execute(CommandLogFunc log) { return impl_->runPlans(log, false); }

GodelBridge::~GodelBridge() = default;

}
