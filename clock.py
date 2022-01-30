from rclpy.node import Node as ROSNode
from std_msgs.msg import Int64


class FakeClock(ROSNode):
    def __init__(self, frequency):
        super(FakeClock, self).__init__('fake_clock')
        self.pub = self.create_publisher(Int64, 'clock', 10)
        self.timer = self.create_timer(frequency, self.publish_clock)
        self.frequency = frequency
        self.count = 0

    def publish_clock(self):
        msg = Int64()
        msg.data = int(self.count)
        self.pub.publish(msg)
        self.count += 1000 * self.frequency
