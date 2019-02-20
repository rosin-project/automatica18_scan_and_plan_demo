#!/usr/bin/env python

import rospy
import godel_msgs.srv as _s
import godel_msgs.msg as _a
from snp_py.snp_helper import (
    init_move_group,
    init_srv,
    init_action,
    call_action,
    unpause_gazebo,
)


class SNPCommander:
    def __init__(self, move_group, planner, delay_unpause_gazebo=0):
        if not rospy.get_param("/sim_robot"):
            raise rospy.ROSException("SNPCommander is only allowed for simulation.")

        # Pause is required to span model with defined joint_values
        unpause_gazebo(delay_unpause_gazebo)

        self._group = init_move_group(move_group)
        self._group.set_planner_id(planner)

        self._surfaceDetection = init_srv("surface_detection", _s.SurfaceDetection)
        self._selectSurface = init_srv("select_surface", _s.SelectSurface)
        self._getAvailableMotionPlan = init_srv(
            "get_available_motion_plans", _s.GetAvailableMotionPlans
        )
        self._plan = init_action("process_planning_as", _a.ProcessPlanningAction)
        self._selectMotionPlan = init_action(
            "select_motion_plan_as", _a.SelectMotionPlanAction
        )
        self._plans = []

    def move_home(self):
        self._group.set_named_target("home")
        return self._group.go(wait=True) is not None

    def surface_detection(self):
        return self._surfaceDetection(
            _s.SurfaceDetectionRequest(
                action=_s.SurfaceDetectionRequest.SCAN_AND_FIND_ONLY,
                use_default_parameters=True,
            )
        ).surfaces_found

    def select_surface(self):
        return self._selectSurface(
            _s.SelectSurfaceRequest(action=_s.SelectSurfaceRequest.SELECT_ALL)
        ).succeeded

    def plan(self, timeout=120):
        return call_action(
            self._plan,
            _a.ProcessPlanningGoal(
                action=_a.ProcessPlanningGoal.GENERATE_MOTION_PLAN_AND_PREVIEW,
                use_default_parameters=True,
            ),
            timeout,
        ).succeeded

    def get_available_motion_plan(self):
        self._plans = self._getAvailableMotionPlan(
            _s.GetAvailableMotionPlansRequest()
        ).names
        return self._plans != []

    def select_motion_plan(self, simulate, timeout=120):
        for p in self._plans:
            result = call_action(
                self._selectMotionPlan,
                _a.SelectMotionPlanGoal(
                    simulate=simulate, wait_for_execution=True, name=p
                ),
                timeout,
            )
            if result.code != _a.SelectMotionPlanResult.SUCCESS:
                return False
        return True

