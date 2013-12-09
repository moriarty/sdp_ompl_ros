#!/usr/bin/env python

PKG = 'sdp_ompl_ros'

import roslib
roslib.load_manifest(PKG)

import sys
import rospy

from sdp_ompl_ros.msg import Path
from sdp_ompl_ros.msg import PlanningEnvironment
from sdp_ompl_ros.msg import PlanningProblem
from sdp_ompl_ros.srv import GetPlan
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

def planning_client():
    rospy.wait_for_service('generic_plannin_server')
    try:
        generic_planner = rospy.ServiceProxy('generic_plannin_server', GetPlan)
        plan = generic_planner()


if __name__ == '__main__':
    planning_client()
