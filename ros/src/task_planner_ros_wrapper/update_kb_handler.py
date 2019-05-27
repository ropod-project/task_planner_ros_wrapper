#! /usr/bin/env python3
''' Handles TaskPlannerUpdateKB service requests.
'''

from __future__ import print_function
import rospy
from task_planner_ros_wrapper.srv import TaskPlannerUpdateKBResponse
from task_planner_ros_wrapper.converter import Converter

class UpdateKBHandler():

    """Handles TaskPlannerUpdateKB service requests.
    
    :kb_interface: KnowledgeBaseInterface object
    """

    def __init__(self, kb_interface):
        self.kb_interface = kb_interface

    def get_safe_response(self, req):
        """Handle individual parts of the update process within try block.

        :req: TaskPlannerUpdateKBRequest
        :returns: TaskPlannerUpdateKBResponse

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
        return TaskPlannerUpdateKBResponse(success=success)
        
    def _handle_facts_to_add(self, facts):
        """Add all predicates to kb

        :facts: list of TaskPlannerPredicate
        :returns: bool

        """
        if not facts:
            return True
        return self.kb_interface.insert_facts(
            [Converter.predicate_ros_to_tuple(fact) for fact in facts])

    def _handle_facts_to_remove(self, facts):
        """Remove all predicates from kb

        :facts: list of TaskPlannerPredicate
        :returns: bool

        """
        if not facts:
            return True
        return self.kb_interface.remove_facts(
            [Converter.predicate_ros_to_tuple(fact) for fact in facts])

    def _handle_fluents_to_add(self, fluents):
        """Add all fluents to kb

        :fluents: list of TaskPlannerFluent
        :returns: bool

        """
        if not fluents:
            return True
        return self.kb_interface.insert_fluents(
            [Converter.fluent_ros_to_tuple(fluent) for fluent in fluents])

    def _handle_fluents_to_remove(self, fluents):
        """Remove all fluents from kb

        :fluents: list of TaskPlannerFluent
        :returns: bool

        """
        if not fluents:
            return True
        success = self.kb_interface.remove_fluents(
            [Converter.fluent_ros_to_tuple(fluent) for fluent in fluents])
        return success

    def _handle_goals_to_add(self, goals):
        """Add all goals to kb

        :goals: list of TaskPlannerPredicate
        :returns: bool

        """
        if not goals:
            return True
        return self.kb_interface.insert_goals(
            [Converter.predicate_ros_to_tuple(goal) for goal in goals])

    def _handle_goals_to_remove(self, goals):
        """Remove all goals from kb

        :goals: list of TaskPlannerPredicate
        :returns: bool

        """
        if not goals:
            return True
        return self.kb_interface.remove_goals(
            [Converter.predicate_ros_to_tuple(goal) for goal in goals])
