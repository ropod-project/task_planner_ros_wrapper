#! /usr/bin/env python3
'''Convert Predicate and Fluent Object defined in task_planner.KnowledgeBaseInterface
to their ROS versions, namely PredicateROS and FluentROS respectively
and vice versa. Also possible to convert ROS versions to tuple.
'''
from ropod_ros_msgs.msg import TaskRequest as TaskRequestROS
from ropod_ros_msgs.msg import Area as AreaROS
from ropod_ros_msgs.msg import SubArea as SubAreaROS
from ropod_ros_msgs.msg import Action as ActionROS
from ropod_ros_msgs.msg import Elevator as ElevatorROS

from diagnostic_msgs.msg import KeyValue
from task_planner.knowledge_base_interface import Predicate, Fluent
from task_planner_ros_wrapper.msg import Predicate as PredicateROS
from task_planner_ros_wrapper.msg import Fluent as FluentROS
from task_planner_ros_wrapper.msg import FluentValue as FluentROSValue

from ropod.structs.task import TaskRequest
from ropod.structs.action import Action
from ropod.structs.area import Area, SubArea

class MessageConverter():

    """Convert ROS versions of Predicate and Fluent to their respective obj and
    vice versa."""

    @staticmethod
    def predicate_obj_to_ros(predicate):
        """
        :predicate: Predicate
        :returns: PredicateROS

        """
        return PredicateROS(
            name=predicate.name,
            params=[KeyValue(key=param.name, value=param.value) for param in predicate.params])

    @staticmethod
    def predicate_ros_to_obj(predicate):
        """
        :predicate: PredicateROS
        :returns: Predicate

        """
        return Predicate.from_dict({
            'name': predicate.name,
            'params': [{'name':param.key, 'value':param.value} for param in predicate.params]})

    @staticmethod
    def predicate_ros_to_tuple(predicate):
        """
        :predicate: PredicateROS
        :returns: tuple ('name', [('key', 'value')])

        """
        return (predicate.name, [(param.key, param.value) for param in predicate.params])

    @staticmethod
    def fluent_obj_to_ros(fluent):
        """
        :fluent: Fluent
        :returns: FluentROS

        """
        return FluentROS(
            name=fluent.name,
            params=[KeyValue(key=param.name, value=param.value) for param in fluent.params],
            value=MessageConverter.fluent_value_obj_to_ros(fluent.value))

    @staticmethod
    def fluent_ros_to_obj(fluent):
        """
        :predicate: FluentROS
        :returns: Fluent

        """
        return Fluent.from_dict({
            'name': fluent.name,
            'params': [{'name':param.key, 'value':param.value} for param in fluent.params],
            'value': MessageConverter.fluent_value_ros_to_obj(fluent.value)})

    @staticmethod
    def fluent_ros_to_tuple(fluent):
        """
        :predicate: FluentROS
        :returns: tuple ('name', [('key', 'value')], value)

        """
        return (fluent.name, [(param.key, param.value) for param in fluent.params], \
                MessageConverter.fluent_value_ros_to_obj(fluent.value))

    @staticmethod
    def fluent_value_obj_to_ros(value):
        """
        :value: string or int
        :returns: FluentROSValue

        """
        if isinstance(value, int):
            return FluentROSValue(
                data_type=FluentROSValue.INT,
                data=str(value))
        if isinstance(value, str):
            return FluentROSValue(
                data_type=FluentROSValue.STRING,
                data=value)
        return FluentROSValue(data_type=FluentROSValue.STRING)

    @staticmethod
    def fluent_value_ros_to_obj(fluent_value):
        """
        :fluent_value: FluentROSValue
        :returns: string or int

        """
        if fluent_value.data_type == FluentROSValue.INT:
            return int(fluent_value.data)
        if fluent_value.data_type == FluentROSValue.STRING:
            return fluent_value.data
        return fluent_value.data

    @staticmethod
    def task_request_obj_to_ros(task_request):
        """
        :task_request: ropod.structs.task/TaskRequest
        :returns: ropod_ros_msgs/TaskRequest

        """
        return TaskRequestROS(
            load_type=task_request.load_type,
            load_id=task_request.load_id,
            user_id=task_request.user_id,
            earliest_start_time=task_request.earliest_start_time,
            latest_start_time=task_request.latest_start_time,
            priority=task_request.priority,
            pickup_pose=MessageConverter.area_obj_to_ros(task_request.pickup_pose),
            delivery_pose=MessageConverter.area_obj_to_ros(task_request.delivery_pose))

    @staticmethod
    def task_request_ros_to_obj(task_request):
        """
        :task_request: ropod_ros_msgs/TaskRequest
        :returns: ropod.structs.task/TaskRequest

        """
        task_req_obj = TaskRequest()
        task_req_obj.load_type = task_request.load_type
        task_req_obj.load_id = task_request.load_id
        task_req_obj.user_id = task_request.user_id
        task_req_obj.earliest_start_time = task_request.earliest_start_time
        task_req_obj.latest_start_time = task_request.latest_start_time
        task_req_obj.pickup_pose = MessageConverter.area_ros_to_obj(task_request.pickup_pose)
        task_req_obj.delivery_pose = MessageConverter.area_ros_to_obj(task_request.delivery_pose)
        task_req_obj.priority = task_request.priority
        return task_req_obj

    @staticmethod
    def area_obj_to_ros(area):
        """
        :area: ropod.structs.area/Area
        :returns: ropod_ros_msgs/Area

        """
        return AreaROS(
            id=area.id,
            name=area.name,
            type=area.type,
            floor_number=area.floor_number,
            sub_areas=[MessageConverter.sub_area_obj_to_ros(sub_area) for sub_area in area.sub_areas])

    @staticmethod
    def area_ros_to_obj(area):
        """
        :area: ropod_ros_msgs/Area
        :returns: ropod.structs.area/Area

        """
        area_obj = Area()
        area_obj.id = area.id
        area_obj.name = area.name
        area_obj.floor_number = area.floor_number
        area_obj.type = area.type
        area_obj.sub_areas = [MessageConverter.sub_area_ros_to_obj(sub_area) \
            for sub_area in area.sub_areas]
        return area_obj

    @staticmethod
    def sub_area_obj_to_ros(sub_area):
        """
        :sub_area: ropod.structs.area/SubArea
        :returns: ropod_ros_msgs/SubArea

        """
        return SubAreaROS(
            id=sub_area.id,
            name=sub_area.name,
            type=sub_area.type,
            capacity=sub_area.capacity)

    @staticmethod
    def sub_area_ros_to_obj(sub_area):
        """
        :sub_area: ropod_ros_msgs/SubArea
        :returns: ropod.structs.area/SubArea

        """
        sub_area_obj = SubArea()
        sub_area_obj.id = sub_area.id
        sub_area_obj.name = sub_area.name
        sub_area_obj.type = sub_area.type
        sub_area_obj.capacity = sub_area.capacity
        return sub_area_obj

    @staticmethod
    def action_obj_to_ros(action):
        """
        :action: ropod.structs.action/Action
        :returns: ropod_ros_msgs/Action

        """
        return ActionROS(
            action_id=action.id,
            type=action.type,
            execution_status=action.execution_status,
            estimated_duration=action.eta,
            start_floor=action.start_floor,
            goal_floor=action.goal_floor,
            areas=[MessageConverter.area_obj_to_ros(area) for area in action.areas],
            sub_areas=[MessageConverter.sub_area_obj_to_ros(sub_area) for sub_area in action.subareas],
            elevator=ElevatorROS(elevator_id=action.elevator_id))

    @staticmethod
    def action_ros_to_obj(action):
        """
        :action: ropod_ros_msgs/Action
        :returns: ropod.structs.action/Action

        """
        action_obj = Action()
        action_obj.id = action.action_id
        action_obj.type = action.type
        action_obj.start_floor = action.start_floor
        action_obj.goal_floor = action.goal_floor
        action_obj.elevator_id = action.elevator.elevator_id
        action_obj.execution_status = action.execution_status
        action_obj.eta = action.estimated_duration
        action_obj.areas = [MessageConverter.area_ros_to_obj(area) for area in action.areas]
        action_obj.subareas = [MessageConverter.area_ros_to_obj(sub_area) for sub_area in action.sub_areas]
        return action_obj
