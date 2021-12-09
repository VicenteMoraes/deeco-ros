from deeco.ros import ROSComponent, ROSSim
from deeco.core import Node
from deeco.plugins.simplenetwork import SimpleNetwork
from deeco.plugins.knowledgepublisher import KnowledgePublisher
from std_msgs.msg import String
import rclpy
import threading


rclpy.init()
sim = ROSSim()
node = Node(sim)
robot = ROSComponent(node, String, 'help')
node.add_component(robot)
SimpleNetwork(sim, delay_ms_mu=20, delay_ms_sigma=5)
KnowledgePublisher(node)

sim.run(15)
