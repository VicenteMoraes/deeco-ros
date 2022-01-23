import rclpy
from std_msgs.msg import Int64
import time
import threading


i = 0
def fake_clock(limit, frequency=1):
    node = rclpy.create_node('clock')
    pub = node.create_publisher(Int64, 'clock', 10)

    def publish_clock():
        global i
        if i >= limit:
            rclpy.shutdown()
        msg = Int64()
        msg.data = int(i)
        pub.publish(msg)
        i += 1000 * frequency

    executor = rclpy.executors.MultiThreadedExecutor()
    timer = node.create_timer(frequency, publish_clock)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

