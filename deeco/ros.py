from deeco.core import Component
import rclpy
from std_msgs.msg import String, Int64
from deeco.sim import Sim, SimScheduler
from time import sleep


class ROSComponent(Component):
    def __init__(self, node, topic):
        super().__init__(node)
        self.topic = topic
        self.ros_node = rclpy.create_node(f"ensemble_{topic}")
        self.ros_sub = self.ros_node.create_subscription(String, topic, self.callback, 10)

    def callback(self, msg):
        self.knowledge.data = msg.data
        print(msg.data)


class ROSScheduler(SimScheduler):
    def __init__(self):
        super().__init__()
        self.ros_node = rclpy.create_node(f"ros_scheduler")
        self.ros_sub = self.ros_node.create_subscription(Int64, '/clock', self.run, 10)
        self.limit_ms = 0

    def set_limit(self, limit_ms):
        self.limit_ms = limit_ms

    def check_if_done(self, time_ms):
        return time_ms > self.limit_ms

    def run(self, clock):
        self.time_ms = int(clock.data / 1e3)
        if self.check_if_done(clock.data):
            rclpy.shutdown()
        else:
            event: Timer = self.events.get()
            event.run(self.time_ms)


class ROSSim(Sim):
    def __init__(self):
        super().__init__()
        self.scheduler = ROSScheduler()

    def run(self, limit_ms):
        self.scheduler.set_limit(limit_ms)
        for plugin in self.plugins:
            plugin.run(self.scheduler)

	# Schedule nodes
        for node in self.nodes:
            node.run(self.scheduler)
        while rclpy.ok():
            pass
        print('All done')
