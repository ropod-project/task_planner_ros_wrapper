#!/usr/bin/python3
from task_planner.knowledge_base_interface import KnowledgeBaseInterface
from task_planner.metric_ff_interface import MetricFFInterface


# based on fleet_management/plugins/planning.py
def initialize_knowledge_base(kb_database_name):
    kb_interface = KnowledgeBaseInterface(kb_database_name)

    print('[initialize_knowledge_base] Initializing elevators')
    elevator_facts = [('elevator_at', [('elevator', 'elevator0'),
                                       ('loc', 'BRSU_A_L0_A8')]),
                      ('elevator_at', [('elevator', 'elevator0'),
                                       ('loc', 'BRSU_A_L2_C1')])]
    kb_interface.insert_facts(elevator_facts)

    elevator_fluents = [('elevator_floor', [('elevator', 'elevator0')], 100)]
    kb_interface.insert_fluents(elevator_fluents)

    elevator_location_fluents = [('location_floor', [('loc', 'BRSU_A_L0_A8')], 0),
                                 ('location_floor', [('loc', 'BRSU_A_L2_C1')], 2)]
    kb_interface.insert_fluents(elevator_location_fluents)

initialize_knowledge_base('ropod_kb')
