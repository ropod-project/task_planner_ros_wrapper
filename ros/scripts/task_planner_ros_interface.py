#! /usr/bin/env python3
'''
ROS wrapper for task planner.
'''

from __future__ import print_function

import rospy
import uuid
from ropod.structs.task import TaskRequest
from task_planner.metric_ff_interface import MetricFFInterface
from task_planner.knowledge_base_interface import KnowledgeBaseInterface

NODE = 'task_planner_ros_interface'

class TaskPlannerROSInterface():

    """ROS wrapper for task planner."""

    def __init__(self):
        planner_params = rospy.get_param('~planner_params', None)
        print(planner_params)
        self.kb_interface = KnowledgeBaseInterface(planner_params['kb_database_name'])
        self.planner = MetricFFInterface(
                planner_params['kb_database_name'],
                planner_params['domain_file'],
                planner_params['planner_cmd'],
                planner_params['plan_file_path'],
                debug=True)
        print("initialised planner")

        task_request = TaskRequest()
        task_request.load_id = 'mobidik_123'
        task_request.load_type = 'mobidik'
        task_request.delivery_pose.id = 'C0'
        task_request.delivery_pose.name = 'BRSU_L0_C0'
        task_request.delivery_pose.floor_number = '0'
        task_request.pickup_pose.id = 'C5'
        task_request.pickup_pose.name = 'BRSU_L0_C5'
        task_request.pickup_pose.floor_number = '0'

        robot_name = 'dummy_robot_{0}'.format(str(uuid.uuid4()))
        self.add_task_request_related_objects_and_fluents(task_request, robot_name)
        actions = []
        try:
            # we set the task goals based on the task request
            task_goals = []
            if task_request.load_type == 'mobidik':
                task_goals = [('load_at', [('load', task_request.load_id),
                                           ('loc', task_request.delivery_pose.name)]),
                              ('empty_gripper', [('bot', robot_name)])]
            elif task_request.load_type == 'sickbed':
                # TBD
                pass

            # we get the action plan
            plan_found, actions = self.planner.plan(task_request, robot_name, task_goals)
            if plan_found:
                for action in actions:
                    print("Action added: %s", action.type)
            else:
                print('Task plan could not be found' )
        except Exception as exc:
            print('A plan could not be created: %s', str(exc))

        # we remove the location of the dummy robot from the knowledge base
        self.kb_interface.remove_facts([('robot_at', [('bot', robot_name),
                                                      ('loc', task_request.pickup_pose.name)]),
                                        ('empty_gripper', [('bot', robot_name)])])

    def add_task_request_related_objects_and_fluents(self, task_request, robot_name, elevator_needed=False):
        """add objects and fluents to the kb using kb interface related to task
        request.

        :task_request: ropod.structs.task.TaskRequest
        :robot_name: string
        :returns: None

        """
        load_id = 'load_' + task_request.load_id

        self.kb_interface.insert_facts([('robot_at', [('bot', robot_name),
                                                      ('loc', task_request.pickup_pose.name)]),
                                        ('load_at', [('load', load_id),
                                                     ('loc', task_request.pickup_pose.name)]),
                                        ('empty_gripper', [('bot', robot_name)])])

        self.kb_interface.insert_fluents([('location_floor',
                                           [('loc', task_request.pickup_pose.name)],
                                           task_request.pickup_pose.floor_number),
                                          ('location_floor',
                                           [('loc', task_request.delivery_pose.name)],
                                           task_request.delivery_pose.floor_number),
                                          ('robot_floor', [('bot', robot_name)],
                                           task_request.pickup_pose.floor_number),
                                          ('load_floor', [('load', load_id)],
                                           task_request.pickup_pose.floor_number)])

        if elevator_needed:
            # copied from fleet management's initialise knowledge base
            elevator_facts = [('elevator_at', [('elevator', 'elevator0'),
                                               ('loc', 'AMK_B_L-1_C2')]),
                              ('elevator_at', [('elevator', 'elevator0'),
                                               ('loc', 'AMK_B_L4_C0')])]
            self.kb_interface.insert_facts(elevator_facts)

            elevator_fluents = [('elevator_floor', [('elevator', 'elevator0')], 100)]
            self.kb_interface.insert_fluents(elevator_fluents)

            elevator_location_fluents = [('location_floor', [('loc', 'AMK_B_L-1_C2')], -1),
                                         ('location_floor', [('loc', 'AMK_B_L4_C0')], 4)]
            self.kb_interface.insert_fluents(elevator_location_fluents)

if __name__ == "__main__":
    rospy.init_node(NODE)
    TASK_PLANNER_ROS_INTERFACE = TaskPlannerROSInterface()
    # rospy.spin()
    rospy.loginfo("[%s] Stopped", NODE)
