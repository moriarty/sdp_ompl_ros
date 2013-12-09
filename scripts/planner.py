#!/usr/bin/env python

PKG = 'sdp_ompl_ros'

import roslib
roslib.load_manifest(PKG)

import sys
import rospy

from sdp_ompl_ros.msg import Event
from sdp_ompl_ros.msg import Path
from sdp_ompl_ros.msg import PlanningEnvironment
from sdp_ompl_ros.msg import PlanningProblem
from sdp_ompl_ros.srv import GetPlan
from KinematicCarMultiPlanner import KinematicCarMultiPlanner
import StringIO as StrIO
import numpy as np
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

TIME = 1.0

def handle_planning_request(req):
    print "some planning whould happen here"

    problem = rospy.wait_for_message('planning_problem', PlanningProblem)
    env = rospy.wait_for_message('planning_environment', PlanningEnvironment)

    demo = KinematicCarMultiPlanner()
    start = problem.start
    goal = problem.goal
    upper = env.bounds.upper
    lower = env.bounds.lower
    demo.setStart(x=start.x, y=start.y, yaw=start.theta)
    demo.setGoal(x=goal.x, y=goal.y, yaw=start.theta)
    demo.setBounds(high=[upper.x, upper.y], low=[lower.x, lower.y])
    demo.setRRTPlanner()

    if demo.solve(TIME):
        path = demo.getPath()
        data = np.loadtxt(StrIO.StringIO(path.printAsMatrix()))
        print data[0:2,:]
        print data[-2:,:]





def planner():
    rospy.init_node('generic_planner_server')
    rospy.Subscriber('planning_req_event', Event, handle_planning_request)
    
    print "Generic Planner Ready"
    rospy.spin()

if __name__ == '__main__':
    planner()
