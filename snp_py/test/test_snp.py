#!/usr/bin/env python
PKG = "snp_py"
NAME = "test_snp"

import sys
import rospy
from moveit_commander import MoveItCommanderException
from snp_py.snp_interface import SNPCommander
import unittest, rostest


class CheckRecordedFile(unittest.TestCase):
    def setUp(self):
        rospy.init_node("snp_test")
        move_group = rospy.get_param("/godel_process_planning/blend_group")
        self.SNP = SNPCommander(move_group, planner="PTP", delay_unpause_gazebo=60)

    def test_snp(self):
        try:
            self.assertTrue(self.SNP.move_home())
            self.assertTrue(self.SNP.surface_detection())
            self.assertTrue(self.SNP.select_surface())
            self.assertTrue(self.SNP.plan())
            self.assertTrue(self.SNP.get_available_motion_plan())
            self.assertTrue(self.SNP.select_motion_plan(simulate=True))
            self.assertTrue(self.SNP.select_motion_plan(simulate=False))
        except (
            MoveItCommanderException,
            rospy.ROSException,
            rospy.ServiceException,
        ) as e:
            rospy.logerr("SNP failed due to %s" % e)


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, CheckRecordedFile, sys.argv)
