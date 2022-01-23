from deeco.core import Component
import rclpy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int64
from deeco.sim import Sim, SimScheduler
import threading


class ROSComponent(Component):
    def __init__(self, node):
        super().__init__(node)
        self.node.runtime.add_ros_component(self)
        self.ros_attributes = {}

    def add_attribute(self, attribute):
        key = attribute.topic
        self.ros_attributes[key] = attribute

    def get_ros_node(self):
        return self.node.runtime.ros_node


class ROSScheduler(SimScheduler):
    def __init__(self, runtime):
        super().__init__()
        self.runtime = runtime
        self.ros_sub = self.runtime.ros_node.create_subscription(Int64, '/clock', self.run, 10)
        self.limit_ms = 0
        self.current_event = None

    def set_limit(self, limit_ms):
        self.limit_ms = limit_ms

    def check_if_done(self, time_ms):
        return time_ms >= self.limit_ms

    def run(self, clock):
        self.time_ms = clock.data
        print(self.time_ms)
        if self.check_if_done(clock.data):
            rclpy.shutdown()
        if self.current_event is None:
            self.current_event = self.events.get()
        if self.current_event.time_ms <= self.time_ms:
            self.current_event.run(self.time_ms)
            self.current_event = None


class ROSSim(Sim):
    def __init__(self):
        super().__init__()
        self.ros_components = []
        self.ros_node = rclpy.create_node('deeco_sim')
        self.scheduler = ROSScheduler(self)

    def add_ros_component(self, ros_component):
        self.ros_components.append(ros_component)

    def run(self, limit_ms):
        self.scheduler.set_limit(limit_ms)
        for plugin in self.plugins:
            plugin.run(self.scheduler)

        # Schedule nodes
        for node in self.nodes:
            node.run(self.scheduler)

        rclpy.spin(self.ros_node)
        print('All done')
