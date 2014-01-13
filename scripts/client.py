#!/usr/bin/env python

#PKG = 'sdp_ompl_ros'

#import roslib
#roslib.load_manifest(PKG)

import sys
import rospy

from sdp_ompl_ros.msg import Bounds
from sdp_ompl_ros.msg import Bound
from sdp_ompl_ros.msg import Obstacle
from sdp_ompl_ros.msg import Path
from sdp_ompl_ros.msg import PlanningEnvironment
from sdp_ompl_ros.msg import PlanningProblem
from sdp_ompl_ros.srv import GetPlan
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

from std_msgs.msg import String
from sdp_ompl_ros.msg import Event

def publish_event(pub, value):
    event = Event()
    event.value = value
    pub.publish(event)
    rospy.loginfo("Event Published")

def publish_environmnet(pub, values):
    env = PlanningEnvironment()

    sqr_obstacle = Obstacle()
    sqr_obstacle.type = "sqaure"
    sqr_obstacle.size = 3.0
    sqr_obstacle.pose.x = 0.0
    sqr_obstacle.pose.y = 0.0
    sqr_obstacle.pose.theta = 0.0
    
    env.bounds.upper.x = 10
    env.bounds.upper.y = 10
    env.bounds.lower.x = -10
    env.bounds.lower.y = -10

    env.obstacles = [sqr_obstacle]
    pub.publish(env)
    rospy.loginfo("Environment Published")    

def publish_problem(pub, values):
    problem = PlanningProblem()
    problem.start.x = -5.0
    problem.start.y = -5.0
    problem.start.theta = 0.0
    problem.goal.x = 5.0
    problem.goal.y = 5.0
    problem.goal.theta = 5.0

    pub.publish(problem)
    rospy.loginfo("Problem Published")

def main():
    event_pub = rospy.Publisher('planning_req_event', Event)
    env_pub = rospy.Publisher('planning_environment', PlanningEnvironment)
    problem_pub = rospy.Publisher('planning_problem', PlanningProblem)

    while not rospy.is_shutdown():
        rospy.init_node('planning_client')

        publish_event(event_pub, "start")
        publish_environmnet(env_pub, "HELLO WORLD")
        publish_problem(problem_pub, "HELLO WORLD")
        
        
        rospy.sleep(10.0)



if __name__ == '__main__':
    main()
