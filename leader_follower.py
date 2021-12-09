from deeco.ros import ROSComponent, ROSSim
from deeco.core import Node, BaseKnowledge
from deeco.plugins.simplenetwork import SimpleNetwork
from deeco.plugins.knowledgepublisher import KnowledgePublisher
from std_msgs.msg import String, Int64
import rclpy



class Leader(ROSComponent):
    class Knowledge(BaseKnowledge):
        def __init__(self):
            super().__init__()
            self.position = None
            self.goal = None


    def __init__(self, node, ros_type=Int64, topic, initial_position, goal):
        super().__init__(node, ros_type, topic)
        self.knowledge = self.Knowledge()
        self.knowledge.position = initial_position
        self.knowledge.goal = goal

