from deeco.core import Component
import rclpy
from std_msgs.msg import String


class ROSComponent(Component):
    def __init__(self, node, topic):
        super().__init__(node)
        self.topic = topic
        self.ros_node = rclpy.create_node(f"ensemble_{topic}")
        self.ros_sub = self.ros_node.create_subscription(String, topic, self.callback, 10)

    def callback(self, msg):
        self.knowledge.data = msg.data
        print(msg.data)
