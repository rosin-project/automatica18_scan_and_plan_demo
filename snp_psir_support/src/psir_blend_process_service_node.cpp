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

#include <ros/ros.h>

#include <godel_msgs/ProcessExecutionAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <industrial_robot_simulator_service/SimulateTrajectory.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <algorithm>

#include <modbus/modbus.h>

void appendTrajectory(trajectory_msgs::JointTrajectory& target, const trajectory_msgs::JointTrajectory& extend) {
  const ros::Duration offset = target.points.empty() ? ros::Duration(0.0) : target.points.back().time_from_start;
  std::transform(extend.points.begin(), extend.points.end(), std::back_inserter(target.points),
  [offset](trajectory_msgs::JointTrajectoryPoint pt){ pt.time_from_start += offset; return pt;});
}

class PsirBlendProcess {
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<godel_msgs::ProcessExecutionAction> as_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
  ros::ServiceClient sim_client_;
  modbus_t *modbus_ {nullptr};
  int spindle_coil_{1};

  bool move(const trajectory_msgs::JointTrajectory &trajectory) {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;

    goal.trajectory.header.stamp = ros::Time(0);
    for (auto &point: goal.trajectory.points) {
        point.accelerations.assign(point.accelerations.size(), 0.0);
    }
    return ac_.sendGoalAndWait(goal, ros::Duration(trajectory.points.back().time_from_start*1.1)) == actionlib::SimpleClientGoalState::SUCCEEDED;
  }
  void execute(const godel_msgs::ProcessExecutionGoalConstPtr &goal) {
    godel_msgs::ProcessExecutionResult res;
    res.success = goal->simulate? simulateProcess(goal) : executeProcess(goal);
    if(res.success) {
      as_.setSucceeded(res);
    }else{
      as_.setAborted(res);
    }
  }

  bool simulateProcess(const godel_msgs::ProcessExecutionGoalConstPtr &goal) {
    industrial_robot_simulator_service::SimulateTrajectory srv;
    srv.request.trajectory = goal->trajectory_approach;
    srv.request.trajectory.header.stamp = ros::Time(0);
    appendTrajectory(srv.request.trajectory, goal->trajectory_process);
    appendTrajectory(srv.request.trajectory, goal->trajectory_depart);
    srv.request.wait_for_execution = goal->wait_for_execution;
    ROS_INFO_STREAM(srv.request);
    return sim_client_.call(srv);
  }
  bool setSpindle(bool on) {
    return modbus_ && modbus_write_bit(modbus_, spindle_coil_, on?1:0) == 1;
  }
  bool executeProcess(const godel_msgs::ProcessExecutionGoalConstPtr &goal) {
    if(!move(goal->trajectory_approach) || !setSpindle(true)) return false;
    bool ok = move(goal->trajectory_process);
    return setSpindle(false) && ok && move(goal->trajectory_depart);
  }

public:
  PsirBlendProcess()
  : as_(nh_,"blend_process_execution_as",boost::bind(&PsirBlendProcess::execute, this, _1), false),
    ac_("joint_trajectory_action", true),
    sim_client_(nh_.serviceClient<industrial_robot_simulator_service::SimulateTrajectory>("simulate_path"))
  {
  }

  ~PsirBlendProcess() {
    if(modbus_) {
      modbus_close(modbus_);
      modbus_free(modbus_);
    }
  }
  bool start() {

    if (!ac_.waitForServer(ros::Duration(30.0))) return false;
    ROS_INFO("WAITED");

    if(!modbus_){
      ros::NodeHandle priv("~");
      std::string spindle_host("169.254.60.1");
      priv.getParam("spindle_host", spindle_host);
      priv.getParam("spindle_coil", spindle_coil_);
      modbus_ = modbus_new_tcp(spindle_host.c_str(), priv.param("spindle_port", 502));
      if (modbus_connect(modbus_) != 0) {
        modbus_free(modbus_);
        modbus_ = nullptr;
        return false;
      }
    }

    as_.start();
    ROS_INFO("Starting PRBT blend process service");
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prbt_blend_process_service_node");

  ros::NodeHandle nh;
  PsirBlendProcess proc;
  if(!proc.start()) return 1;
  ros::spin();
  return 0;
}
