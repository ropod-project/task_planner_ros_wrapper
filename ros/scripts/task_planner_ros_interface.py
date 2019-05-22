#! /usr/bin/env python3
'''
ROS wrapper for task planner.
'''

from __future__ import print_function

import rospy
from ropod.structs.task import TaskRequest
from task_planner.metric_ff_interface import MetricFFInterface

NODE = 'task_planner_ros_interface'

def main():
    planner_name = ''
    domain_file = ''
    planner_cmd = ''
    plan_file_path = ''
    with open('config/planner_config.yaml', 'r') as config_file:
        planner_config = yaml.load(config_file)
        planner_name = planner_config['planner_name']
        domain_file = planner_config['domain_file']
        planner_cmd = planner_config['planner_cmd']
        plan_file_path = planner_config['plan_file_path']

    planner = MetricFFInterface('ropod_kb', domain_file, planner_cmd, plan_file_path, debug=True)

    task_request = TaskRequest()
    task_request.load_id = 'mobidik_123'
    task_request.delivery_pose.id = 'BRSU_L0_C0'

    robot_name = 'frank'
    task_goals = [('load_at', [('load', task_request.load_id),
                               ('loc', task_request.delivery_pose.id)]),
                  ('empty_gripper', [('bot', robot_name)])]
    plan_found, plan = planner.plan(task_request, robot_name, task_goals)

class TaskPlannerROSInterface():

    """ROS wrapper for task planner."""

    def __init__(self):
        pass

if __name__ == "__main__":
    rospy.init_node(NODE)
    TASK_PLANNER_ROS_INTERFACE = TaskPlannerROSInterface()
    rospy.spin()
    rospy.loginfo("[%s] Stopped", NODE)
