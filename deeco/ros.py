from deeco.core import Component
import rclpy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int64
from deeco.sim import Sim, SimScheduler
from time import sleep
import threading


class ROSComponent(Component):
    def __init__(self, node, ros_type, topic, callback=None):
        super().__init__(node)
        self.node.runtime.add_ros_node(self)
        self.topic = topic
        self.ros_node = rclpy.create_node(f"ensemble_{topic}")
        if callback is None:
            callback = self.callback
        self.ros_sub = self.ros_node.create_subscription(ros_type, topic, callback, 10)

    def callback(self, msg):
        self.knowledge.data = msg.data


class ROSScheduler(SimScheduler):
    def __init__(self):
        super().__init__()
        self.ros_node = rclpy.create_node(f"ros_scheduler")
        self.ros_sub = self.ros_node.create_subscription(Int64, '/clock', self.run, 10)
        self.limit_ms = 0
        self.is_running = False
        self.current_event = None

    def set_limit(self, limit_ms):
        self.limit_ms = limit_ms

    def check_if_done(self, time_ms):
        return time_ms > self.limit_ms

    def run(self, clock):
        self.time_ms = int(clock.data / 1e3)
        if self.check_if_done(clock.data):
            rclpy.shutdown()
        elif not self.is_running:
            if self.current_event is None:
                self.current_event = self.events.get()
            if self.current_event.time_ms <= self.time_ms:
                self.is_running = True
                self.current_event.run(self.time_ms)
                self.current_event = None
                self.is_running = False


class ROSSim(Sim):
    def __init__(self):
        super().__init__()
        self.scheduler = ROSScheduler()
        self.ros_nodes = []
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)

    def add_ros_node(self, ros_node):
        self.ros_nodes.append(ros_node)

    def start_ros_executor(self):
        self.executor.add_node(self.scheduler.ros_node)
        for node in self.ros_nodes:
            self.executor.add_node(node.ros_node)
        self.executor_thread.start()

    def run(self, limit_ms):
        self.scheduler.set_limit(limit_ms)
        for plugin in self.plugins:
            plugin.run(self.scheduler)

	# Schedule nodes
        for node in self.nodes:
            node.run(self.scheduler)

        self.start_ros_executor()
        while rclpy.ok():
            pass
        print('All done')
        self.executor_thread.join()
