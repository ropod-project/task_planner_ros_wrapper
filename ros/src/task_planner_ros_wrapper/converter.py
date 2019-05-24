#! /usr/bin/env python3
'''Convert Predicate and Fluent Object defined in task_planner.KnowledgeBaseInterface
to their ROS versions, namely TaskPlannerPredicate and TaskPlannerFluent respectively
and vice versa. Also possible to convert ROS versions to tuple.
'''
from diagnostic_msgs.msg import KeyValue
from task_planner.knowledge_base_interface import Predicate, Fluent, PredicateParams
from task_planner_ros_wrapper.msg import TaskPlannerPredicate, TaskPlannerFluent, TaskPlannerFluentValue

class Converter(object):

    """Convert ROS versions of Predicate and Fluent to their respective obj and
    vice versa."""

    @staticmethod
    def predicate_obj_to_ros(predicate):
        """
        :predicate: Predicate
        :returns: TaskPlannerPredicate

        """
        return TaskPlannerPredicate(
            name=predicate.name, 
            params=[KeyValue(key=param.name, value=param.value) for param in predicate.params])

    @staticmethod
    def predicate_ros_to_obj(predicate):
        """
        :predicate: TaskPlannerPredicate
        :returns: Predicate

        """
        return Predicate.from_dict({
            'name': predicate.name,
            'params': [{'name':param.key, 'value':param.value} for param in predicate.params]})

    @staticmethod
    def predicate_ros_to_tuple(predicate):
        """
        :predicate: TaskPlannerPredicate
        :returns: tuple ('name', [('key', 'value')])

        """
        return (predicate.name, [(param.key, param.value) for param in predicate.params])

    @staticmethod
    def fluent_obj_to_ros(fluent):
        """ 
        :fluent: Fluent
        :returns: TaskPlannerFluent

        """
        return TaskPlannerFluent(
            name=fluent.name, 
            params=[KeyValue(key=param.name, value=param.value) for param in fluent.params],
            value=Converter.fluent_value_obj_to_ros(fluent.value))

    @staticmethod
    def fluent_ros_to_obj(fluent):
        """
        :predicate: TaskPlannerFluent
        :returns: Fluent

        """
        return Fluent.from_dict({
            'name': fluent.name,
            'params': [{'name':param.key, 'value':param.value} for param in fluent.params],
            'value': Converter.fluent_value_ros_to_obj(fluent.value)})

    @staticmethod
    def fluent_ros_to_tuple(fluent):
        """
        :predicate: TaskPlannerFluent
        :returns: tuple ('name', [('key', 'value')], value)

        """
        return (fluent.name, [(param.key, param.value) for param in fluent.params], \
                Converter.fluent_value_ros_to_obj(fluent.value))

    @staticmethod
    def fluent_value_obj_to_ros(value):
        """ 
        :value: string or int
        :returns: TaskPlannerFluentValue

        """
        if isinstance(value, int):
            return TaskPlannerFluentValue(
                data_type=TaskPlannerFluentValue.INT,
                data=str(value))
        elif isinstance(value, str):
            return TaskPlannerFluentValue(
                data_type=TaskPlannerFluentValue.STRING,
                data=value)

    @staticmethod
    def fluent_value_ros_to_obj(fluent_value):
        """ 
        :fluent_value: TaskPlannerFluentValue
        :returns: string or int

        """
        if fluent_value.data_type == TaskPlannerFluentValue.INT:
            return int(fluent_value.data)
        elif fluent_value.data_type == TaskPlannerFluentValue.STRING:
            return fluent_value.data
