from deeco.roscomponent import ROSComponent
from deeco.core import Node
from deeco.sim import Sim
from deeco.plugins.simplenetwork import SimpleNetwork
from deeco.plugins.knowledgepublisher import KnowledgePublisher
import rclpy


rclpy.init()
sim = Sim()
node = Node(sim)
SimpleNetwork(sim, delay_ms_mu=20, delay_ms_sigma=5)
robot = ROSComponent(node, 'help')
node.add_component(robot)
KnowledgePublisher(node)
sim.run(10000)
