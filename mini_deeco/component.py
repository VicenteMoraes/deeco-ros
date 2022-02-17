import shortuuid
import rclpy
from rclpy.action import ActionServer
from knowledge import BaseKnowledge
from std_msgs.msg import String
import json
from deeco_actions.action import KnowledgeExchange


class Component:
    class Knowledge(BaseKnowledge):
        pass

    def __init__(self, sim, name: str = "", server_name: str = "", frequency: float = 0, callback = None):
        self.frequency = frequency if frequency else sim.frequency
        self.ros_node = sim
        self.name = name
        self._uuid = shortuuid.uuid()
        self.server_name = server_name if server_name else f'action_{self._uuid}'
        self.callback = callback if callback is not None else self.execute_goal

        self._action_server = ActionServer(
            self.ros_node,
            KnowledgeExchange,
            self.server_name,
            self.callback
        )

    def knowledge_to_json(self):
        return json.dumps(self.knowledge.__dict__)

    def execute_goal(self, goal_handle):
        result = KnowledgeExchange.Result()
        return result