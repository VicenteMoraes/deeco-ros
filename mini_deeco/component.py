import shortuuid
import rclpy
from rclpy.action import ActionServer
from knowledge import BaseKnowledge
from std_msgs.msg import String
import json
from deeco_actions.action import KnowledgeExchange
from abc import abstractmethod


class Component:
    class Knowledge(BaseKnowledge):
        pass

    def __init__(self, sim, name: str = "", server_name: str = "", frequency: float = 0):
        self.frequency = frequency if frequency else sim.frequency
        self.ros_node = sim
        self.name = name
        self._uuid = shortuuid.uuid()
        self.server_name = server_name if server_name else f'action_{self._uuid}'

        self._action_server = ActionServer(
            self.ros_node,
            KnowledgeExchange,
            self.server_name,
            self.execute_goal
        )

    @staticmethod
    def knowledge_to_json(knowledge):
        return json.dumps(knowledge.__dict__)

    @abstractmethod
    def execute_goal(self, goal_handle):
        pass