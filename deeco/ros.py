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
        self.ros_node = rclpy.create_node(f"ensemble_{self.uuid}")
        self.ros_attributes = {}

    def add_attribute(self, attribute):
        key = attribute.topic
        self.ros_attributes[key] = attribute

    def get_ros_node(self):
        return self.ros_node


class ROSScheduler(SimScheduler):
    def __init__(self, runtime):
        super().__init__()
        self.runtime = runtime
        self.runtime.add_ros_component(self)
        self.ros_node = rclpy.create_node("deeco_scheduler")
        self.ros_sub = self.ros_node.create_subscription(Int64, '/clock', self.run, 10)
        self.limit_ms = 0
        self.current_event = None

    def set_limit(self, limit_ms):
        self.limit_ms = limit_ms

    def check_if_done(self, time_ms):
        return time_ms >= self.limit_ms

    def run(self, clock):
        self.time_ms = clock.data
        if self.check_if_done(self.time_ms):
            rclpy.shutdown()
        if self.events.queue[0].time_ms <= self.time_ms:
            self.current_event = self.events.get()
            self.current_event.run(self.time_ms)


class ROSSim(Sim):
    def __init__(self):
        super().__init__()
        self.ros_components = []
        self.scheduler = ROSScheduler(self)
        self.executor = rclpy.executors.SingleThreadedExecutor()

    def add_ros_component(self, *components):
        for component in components:
            self.ros_components.append(component)

    def run_plugins(self):
        for plugin in self.plugins:
            plugin.run(self.scheduler)

    def schedule_nodes(self):
        for node in self.nodes:
            node.run(self.scheduler)

    def run_ros(self):
        for component in self.ros_components:
            try:
                self.executor.add_node(component.ros_node)
            except AttributeError:
                self.executor.add_node(component)
        self.executor.spin()

    def run(self, limit_ms):
        self.scheduler.set_limit(limit_ms)

        self.run_plugins()
        self.schedule_nodes()
        self.run_ros()

        print('All done')
