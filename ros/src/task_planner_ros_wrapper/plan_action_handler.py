#! /usr/bin/env python3
''' Handles Plan action requests.
'''

from __future__ import print_function
from task_planner_ros_wrapper.msg import PlanGoal, PlanResult
from task_planner_ros_utils.message_converter import MessageConverter

class PlanActionHandler():

    """Handles Plan action requests.

    :planner: TaskPlannerInterface object or its children (for example MetricFFInterface)
    """

    def __init__(self, planner):
        self.planner = planner

    def get_safe_result(self, req):
        """Handle plan action call within try block.

        :req: PlanGoal
        :returns: PlanResult

        """
        plan_found = False
        actions = []
        try:
            plan_found, action_objs = self.planner.plan(
                MessageConverter.task_request_ros_to_obj(req.task_request),
                req.robot_name,
                [MessageConverter.predicate_ros_to_obj(goal) for goal in req.task_goals])
            actions = [MessageConverter.action_obj_to_ros(action) for action in action_objs]
        except Exception as e:
            rospy.logerr(str(e))
        return PlanResult(plan_found=plan_found, actions=actions)
