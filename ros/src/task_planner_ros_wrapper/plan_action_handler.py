#! /usr/bin/env python3
''' Handles TaskPlannerPlan action requests.
'''

from __future__ import print_function
import rospy
from task_planner_ros_wrapper.msg import TaskPlannerPlanGoal, TaskPlannerPlanResult
from task_planner_ros_wrapper.converter import Converter

class PlanActionHandler():

    """Handles TaskPlannerPlan action requests.
    
    :planner: TaskPlannerInterface object or its children (for example MetricFFInterface)
    """

    def __init__(self, planner):
        self.planner = planner

    def get_safe_result(self, req):
        """Handle plan action call within try block.

        :req: TaskPlannerPlanGoal
        :returns: TaskPlannerPlanResult

        """
        plan_found = False
        actions = []
        try:
            plan_found, action_objs = self.planner.plan(
                Converter.task_request_ros_to_obj(req.task_request),
                req.robot_name, 
                [Converter.predicate_ros_to_obj(goal) for goal in req.task_goals])
            actions = [Converter.action_obj_to_ros(action) for action in action_objs]
        except Exception as e:
            print(str(e))
        return TaskPlannerPlanResult(plan_found=plan_found, actions=actions)
