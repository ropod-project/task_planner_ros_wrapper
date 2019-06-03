#! /usr/bin/env python3
''' Handles knowledge base related service requests.
'''

from __future__ import print_function
from task_planner_ros_wrapper.srv import UpdateKBResponse
from task_planner_ros_wrapper.srv import QueryKB, QueryKBRequest, QueryKBResponse
from task_planner_ros_utils.message_converter import MessageConverter

class KBHandler():

    """Handles Knowledge base realated service requests.

    :kb_interface: KnowledgeBaseInterface object
    """

    def __init__(self, kb_interface):
        self.kb_interface = kb_interface

    def get_safe_update_response(self, req):
        """Handle individual parts of the update process within try block.

        :req: UpdateKBRequest
        :returns: UpdateKBResponse

        """
        success = True
        try:
            success = success and self._handle_facts_to_add(req.facts_to_add)
            success = success and self._handle_facts_to_remove(req.facts_to_remove)
            success = success and self._handle_fluents_to_add(req.fluents_to_add)
            success = success and self._handle_fluents_to_remove(req.fluents_to_remove)
            success = success and self._handle_goals_to_add(req.goals_to_add)
            success = success and self._handle_goals_to_remove(req.goals_to_remove)
        except Exception as e:
            pass
        return UpdateKBResponse(success=success)

    def get_safe_query_response(self, req):
        """Handle a query based on the query type within try block

        :req: QueryKBRequest
        :returns: QueryKBResponse

        """
        try:
            if req.query_type == QueryKBRequest.GET_PREDICATE_NAMES:
                return self._handle_get_predicate_names()
            if req.query_type == QueryKBRequest.GET_FLUENT_NAMES:
                return self._handle_get_fluent_names()
            if req.query_type == QueryKBRequest.GET_PREDICATE_ASSERTIONS:
                return self._handle_get_predicate_assertions(req.predicate_name)
            if req.query_type == QueryKBRequest.GET_FLUENT_ASSERTIONS:
                return self._handle_get_fluent_assertions()
            if req.query_type == QueryKBRequest.GET_FLUENT_VALUE:
                return self._handle_get_fluent_value(req.fluent)
        except Exception as e:
            rospy.logerr(str(e))
        return QueryKBResponse()

    def _handle_facts_to_add(self, facts):
        """Add all predicates to kb

        :facts: list of TaskPlannerPredicate
        :returns: bool

        """
        if not facts:
            return True
        return self.kb_interface.insert_facts(
            [MessageConverter.predicate_ros_to_tuple(fact) for fact in facts])

    def _handle_facts_to_remove(self, facts):
        """Remove all predicates from kb

        :facts: list of TaskPlannerPredicate
        :returns: bool

        """
        if not facts:
            return True
        return self.kb_interface.remove_facts(
            [MessageConverter.predicate_ros_to_tuple(fact) for fact in facts])

    def _handle_fluents_to_add(self, fluents):
        """Add all fluents to kb

        :fluents: list of TaskPlannerFluent
        :returns: bool

        """
        if not fluents:
            return True
        return self.kb_interface.insert_fluents(
            [MessageConverter.fluent_ros_to_tuple(fluent) for fluent in fluents])

    def _handle_fluents_to_remove(self, fluents):
        """Remove all fluents from kb

        :fluents: list of TaskPlannerFluent
        :returns: bool

        """
        if not fluents:
            return True
        success = self.kb_interface.remove_fluents(
            [MessageConverter.fluent_ros_to_tuple(fluent) for fluent in fluents])
        return success

    def _handle_goals_to_add(self, goals):
        """Add all goals to kb

        :goals: list of TaskPlannerPredicate
        :returns: bool

        """
        if not goals:
            return True
        return self.kb_interface.insert_goals(
            [MessageConverter.predicate_ros_to_tuple(goal) for goal in goals])

    def _handle_goals_to_remove(self, goals):
        """Remove all goals from kb

        :goals: list of TaskPlannerPredicate
        :returns: bool

        """
        if not goals:
            return True
        return self.kb_interface.remove_goals(
            [MessageConverter.predicate_ros_to_tuple(goal) for goal in goals])

    def _handle_get_predicate_names(self):
        """
        :returns: QueryKBResponse

        """
        return QueryKBResponse(predicate_names=self.kb_interface.get_predicate_names())

    def _handle_get_fluent_names(self):
        """
        :returns: QueryKBResponse

        """
        return QueryKBResponse(fluent_names=self.kb_interface.get_fluent_names())

    def _handle_get_predicate_assertions(self, predicate_name):
        """

        :predicate_name: string
        :returns: QueryKBResponse

        """
        if predicate_name == '':
            predicate_name = None
        predicate_assertions = [MessageConverter.predicate_obj_to_ros(predicate) \
                for predicate in self.kb_interface.get_predicate_assertions(predicate_name)]
        return QueryKBResponse(predicate_assertions=predicate_assertions)

    def _handle_get_fluent_assertions(self):
        """
        :returns: QueryKBResponse

        """
        fluent_assertions = [MessageConverter.fluent_obj_to_ros(fluent) \
            for fluent in self.kb_interface.get_fluent_assertions()]
        return QueryKBResponse(fluent_assertions=fluent_assertions)

    def _handle_get_fluent_value(self, fluent):
        """

        :fluent: task_planner/Fluent
        :returns: QueryKBResponse

        """
        fluent_value = self.kb_interface.get_fluent_value(MessageConverter.fluent_ros_to_tuple(fluent))
        fluent_value_ros = MessageConverter.fluent_value_obj_to_ros(fluent_value)
        return QueryKBResponse(fluent_value=fluent_value_ros)
