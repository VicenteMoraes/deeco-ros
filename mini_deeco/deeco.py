import rclpy
from rclpy.node import Node as ROSNode


class Simulation(ROSNode):
    def __init__(self, time_limit: float, frequency: float = 0.1):
        super(Simulation, self).__init__("deeco_simulation")
        self.frequency = frequency
        self.time_limit = time_limit

        self._ros_executor = rclpy.executors.SingleThreadedExecutor()
        self._ros_executor.add_node(self)
        self.create_timer(self.time_limit, self.end)

    def add_nodes_to_executor(self, *nodes):
        for node in nodes:
            self.executor.add_node(node)

    def end(self):
        print('\nSimulation Ended')
        rclpy.shutdown()

    def start(self):
        self.executor.spin()