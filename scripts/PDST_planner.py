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
from geometry_msgs.msg import Pose2D
from KinematicCarMultiPlanner import KinematicCarMultiPlanner
import StringIO as StrIO
import numpy as np
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

TIME = 1.0

class PDSTPlanner:
    def __init__(self):
        rospy.init_node('pdst_planner_server')
        self.path_pub = rospy.Publisher('pdst_solution_path', Path)
        self.demo = KinematicCarMultiPlanner()

    def subscribe_to_requests(self, topic):
        rospy.Subscriber(topic, Event, self.handle_planning_request)
        rospy.spin()

    def handle_planning_request(self, req):
        # This could be a problem. 
        problem = rospy.wait_for_message('planning_problem', PlanningProblem)
        env = rospy.wait_for_message('planning_environment', PlanningEnvironment)

        start = problem.start
        goal = problem.goal
        upper = env.bounds.upper
        lower = env.bounds.lower
        self.demo.setStart(x=start.x, y=start.y, yaw=start.theta)
        self.demo.setGoal(x=goal.x, y=goal.y, yaw=start.theta)
        self.demo.setBounds(high=[upper.x, upper.y], low=[lower.x, lower.y])
        self.demo.setPDSTPlanner()
        
        if self.demo.solve(TIME):
            path = self.demo.getPath()
            data = np.loadtxt(StrIO.StringIO(path.printAsMatrix()))
            print data[0:2,:]
            print data[-2:,:]
            self.publish_path(self.path_pub, data)

    def publish_path(self, pub, data):
        env = PlanningEnvironment()
        poses = [Pose2D(data[i][0], data[i][1], data[i][2]) for i in xrange(data.shape[0])]
        path = Path(poses)
        pub.publish(path)
        rospy.loginfo("Path Published")  



if __name__ == '__main__':
    planner = PDSTPlanner()
    planner.subscribe_to_requests('planning_req_event')