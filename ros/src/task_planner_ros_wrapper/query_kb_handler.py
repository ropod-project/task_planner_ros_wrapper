#! /usr/bin/env python3
''' Handles TaskPlannerQueryKB service requests.
'''

from __future__ import print_function
from task_planner_ros_wrapper.srv import TaskPlannerQueryKB, TaskPlannerQueryKBRequest, TaskPlannerQueryKBResponse
from task_planner_ros_utils.message_converter import MessageConverter

class QueryKBHandler():

    """Handles TaskPlannerQueryKB service requests.

    :kb_interface: KnowledgeBaseInterface object
    """

    def __init__(self, kb_interface):
        self.kb_interface = kb_interface

    def get_safe_response(self, req):
        """Handle a query based on the query type within try block

        :req: TaskPlannerQueryKBRequest
        :returns: TaskPlannerQueryKBResponse

        """
        try:
            if req.query_type == TaskPlannerQueryKBRequest.GET_PREDICATE_NAMES:
                return self._handle_get_predicate_names()
            if req.query_type == TaskPlannerQueryKBRequest.GET_FLUENT_NAMES:
                return self._handle_get_fluent_names()
            if req.query_type == TaskPlannerQueryKBRequest.GET_PREDICATE_ASSERTIONS:
                return self._handle_get_predicate_assertions(req.predicate_name)
            if req.query_type == TaskPlannerQueryKBRequest.GET_FLUENT_ASSERTIONS:
                return self._handle_get_fluent_assertions()
            if req.query_type == TaskPlannerQueryKBRequest.GET_FLUENT_VALUE:
                return self._handle_get_fluent_value(req.fluent)
        except Exception as e:
            rospy.logerr(str(e))
        return TaskPlannerQueryKBResponse()

    def _handle_get_predicate_names(self):
        """
        :returns: TaskPlannerQueryKBResponse

        """
        return TaskPlannerQueryKBResponse(predicate_names=self.kb_interface.get_predicate_names())

    def _handle_get_fluent_names(self):
        """
        :returns: TaskPlannerQueryKBResponse

        """
        return TaskPlannerQueryKBResponse(fluent_names=self.kb_interface.get_fluent_names())

    def _handle_get_predicate_assertions(self, predicate_name):
        """

        :predicate_name: string
        :returns: TaskPlannerQueryKBResponse

        """
        if predicate_name == '':
            predicate_name = None
        predicate_assertions = [MessageConverter.predicate_obj_to_ros(predicate) \
                for predicate in self.kb_interface.get_predicate_assertions(predicate_name)]
        return TaskPlannerQueryKBResponse(predicate_assertions=predicate_assertions)

    def _handle_get_fluent_assertions(self):
        """
        :returns: TaskPlannerQueryKBResponse

        """
        fluent_assertions = [MessageConverter.fluent_obj_to_ros(fluent) \
            for fluent in self.kb_interface.get_fluent_assertions()]
        return TaskPlannerQueryKBResponse(fluent_assertions=fluent_assertions)

    def _handle_get_fluent_value(self, fluent):
        """

        :fluent: task_planner/Fluent
        :returns: TaskPlannerQueryKBResponse

        """
        fluent_value = self.kb_interface.get_fluent_value(MessageConverter.fluent_ros_to_tuple(fluent))
        fluent_value_ros = MessageConverter.fluent_value_obj_to_ros(fluent_value)
        return TaskPlannerQueryKBResponse(fluent_value=fluent_value_ros)
