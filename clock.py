import rclpy
from std_msgs.msg import Int64
import time


i = 0
rclpy.init()
node = rclpy.create_node('clock')
pub = node.create_publisher(Int64, 'clock', 10)
def publish_clock():
    global i
    msg = Int64()
    msg.data = i
    pub.publish(msg)
    i += 1


timer = node.create_timer(1, publish_clock)
rclpy.spin(node)

