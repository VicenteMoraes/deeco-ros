from deeco.ros import ROSComponent, ROSSim
from deeco.core import Node
from deeco.plugins.simplenetwork import SimpleNetwork
from deeco.plugins.knowledgepublisher import KnowledgePublisher
from deeco.plugins.attributes import ROSString
import rclpy
from clock import fake_clock


rclpy.init()
sim = ROSSim()
node = Node(sim)
robot = ROSComponent(node, 'robot')
str_attribute = ROSString(node, robot, 'topicname')
robot.add_attribute(str_attribute)
node.add_component(robot)
SimpleNetwork(sim, delay_ms_mu=20, delay_ms_sigma=5)
KnowledgePublisher(node)

fake_clock()
sim.run(15e3)
