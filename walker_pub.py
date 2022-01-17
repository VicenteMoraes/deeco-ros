import rclpy
import threading
from geometry_msgs.msg import Pose


def euclidean_distance(nodeA, nodeB):
    return ((nodeA.position.x - nodeB.position.x) ** 2 + (nodeA.position.y - nodeB.position.y) ** 2) ** 0.5


def dist_to(componentA, componentB):
    return euclidean_distance(componentA.knowledge.position, componentB.knowledge.position)


class ROSWalker:
    DEFAULT_SPEED_M_S = 5 / 3.6
    DEFAULT_STEP_MS = 100

    def __init__(self, ros_component, initial_position, topic, speed_ms=DEFAULT_SPEED_M_S, frequency=1):
        self.ros_component = ros_component
        self.pose = initial_position
        self.speed_ms = speed_ms
        self.frequency = frequency
        self.ros_node = rclpy.create_node(f"walker_{topic}")
        self.ros_pub = self.ros_node.create_publisher(Pose, topic, 10)
        self.timer = self.ros_node.create_timer(frequency, self.move)
        self.target = None

    def update_target(self):
        self.target = self.ros_component.knowledge.goal

    def move(self):
        step = self.speed_ms / (1000 / self.DEFAULT_STEP_MS)
        if self.pose is None or self.target is None:
            return

        if euclidean_distance(self.pose, self.target) <= step:
            print('hi')
            self.pose = self.target
        else:
            vector = Pose()
            vector.position.x = self.target.position.x - self.pose.position.x
            vector.position.y = self.target.position.y - self.pose.position.y
            mag = (vector.position.x ** 2 + vector.position.y ** 2) ** 0.5
            vector.position.x *= step / mag
            vector.position.y *= step / mag
            self.pose.position.x += vector.position.x
            self.pose.position.y += vector.position.y

    def pose_pub(self):
        self.update_target()
        self.move()
        self.ros_pub.publish(self.pose)


def start_walkers(walkers: list):
    executor = rclpy.executors.MultiThreadedExecutor()
    for walker in walkers:
        walker.ros_node.create_timer(walker.frequency, walker.pose_pub)
        executor.add_node(walker.ros_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
