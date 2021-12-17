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
        self.ros_component.knowledge.position.x = x
        self.ros_component.knowledge.position.y = y
        self.ros_component.knowledge.position.z = z
        self.ros_component.knowledge.orientation.x = x
        self.ros_component.knowledge.orientation.y = x
        self.ros_component.knowledge.orientation.z = z
        self.ros_component.knowledge.orientation.w = w
