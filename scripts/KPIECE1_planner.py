#!/usr/bin/env python
import rospy

# messages and services
from sdp_ompl_ros.msg import Event
from sdp_ompl_ros.msg import Path
from sdp_ompl_ros.msg import PlanningEnvironment
from sdp_ompl_ros.msg import PlanningProblem
from geometry_msgs.msg import Pose2D
from sdp_ompl_ros.srv import GetPlan

# Config
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from sdp_ompl_ros.cfg import kpiece1_planner_paramsConfig as KPIECE1Config

# custom libraries
import ompl_demo as ompl_d

# Standard Python Libraries
import StringIO as StrIO
import numpy as np
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

TIME = 1.0

class KPIECE1Planner:
    def __init__(self):
        self.path_pub = rospy.Publisher('kpiece1_solution_path', Path)
        #self.goal_bias = None
        #self.border_fraction = None
        self.server = DynamicReconfigureServer(KPIECE1Config, self.reconfigure)
        self.demo = ompl_d.KinematicCarMultiPlanner()

    def reconfigure(self, config, level):
        self.goal_bias = config["goal_bias"]
        self.border_fraction = config["border_fraction"]
        return config

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
        self.demo.setKPIECE1Planner(goal_bias=self.goal_bias, border_fraction=self.border_fraction)
        
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
    rospy.init_node('kpiece1_planner_server')
    planner = KPIECE1Planner()
    planner.subscribe_to_requests('planning_req_event')
