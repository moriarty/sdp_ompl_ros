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


def handle_planning_request(req):
    print "some planning whould happen here"
    return GetPlanResponse()



def planner():
    rospy.init_node('generic_planner_server')
    s = rospy.Service('generic_planner_server', GetPlan,
        handle_planning_request)
    print "Generic Planner Ready"
    rospy.spin()

    pass

if __name__ == '__main__':
    try:
        planner()
    except rospy.ROSInterruptExecption:
        pass
