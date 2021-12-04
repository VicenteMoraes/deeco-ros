from deeco.ros import ROSComponent, ROSSim
from deeco.core import Node
from deeco.plugins.simplenetwork import SimpleNetwork
from deeco.plugins.knowledgepublisher import KnowledgePublisher
import rclpy
import threading


rclpy.init()
sim = ROSSim()
node = Node(sim)
robot = ROSComponent(node, 'help')
node.add_component(robot)
SimpleNetwork(sim, delay_ms_mu=20, delay_ms_sigma=5)
KnowledgePublisher(node)

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(sim.scheduler.ros_node)
executor.add_node(robot.ros_node)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

sim.run(10000)
executor_thread.join()
