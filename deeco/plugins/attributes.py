from deeco.core import NodePlugin
from std_msgs.msg import String, Int64
from geometry_msgs.msg import Pose


class Attribute(NodePlugin):
    def __init__(self, node):
        super().__init__(node)


class ROSAttribute(Attribute):
    def __init__(self, node, ros_component, ros_type, topic, callback=None):
        super().__init__(node)
        self.ros_component = ros_component
        self.topic = topic
        if callback is None:
            callback = self.callback
        self.sub = ros_component.ros_node.create_subscription(ros_type, topic, callback, 10)

    def callback(self, msg):
        self.ros_component.knowledge.data = msg.data


class ROSString(ROSAttribute):
    def __init__(self, node, ros_component, topic):
        ros_type = String
        super().__init__(node, ros_component, ros_type, topic)

class ROSInt64(ROSAttribute):
    def __init__(self, node, ros_component, topic):
        ros_type = Int64
        super().__init__(node, ros_component, ros_type, topic)

class ROSPose(ROSAttribute):
    def __init__(self, node, ros_component, topic):
        ros_type = Pose
        super().__init__(node, ros_component, ros_type, topic, callback=self.pose_callback)

    def pose_callback(self, msg):
        self.ros_component.knowledge.position.position.x = msg.position.x
        self.ros_component.knowledge.position.position.y = msg.position.y
        self.ros_component.knowledge.position.position.z = msg.position.z
        self.ros_component.knowledge.position.orientation.x = msg.orientation.x
        self.ros_component.knowledge.position.orientation.y = msg.orientation.y
        self.ros_component.knowledge.position.orientation.z = msg.orientation.z
        self.ros_component.knowledge.position.orientation.w = msg.orientation.w
