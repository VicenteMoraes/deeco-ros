from deeco.ros import ROSComponent, ROSSim
from deeco.plugins.ensemblereactor import EnsembleMember, EnsembleReactor, has_member
from deeco.core import EnsembleDefinition, process
from deeco.core import Node, BaseKnowledge, ComponentRole
from deeco.plugins.simplenetwork import SimpleNetwork
from deeco.plugins.knowledgepublisher import KnowledgePublisher
from deeco.mapping import SetValue
from geometry_msgs.msg import Pose
from deeco.plugins.simplenetwork import SimpleRangeLimitedNetwork
from deeco.plugins.walker import Walker
from deeco.plugins.snapshoter import Snapshoter
from deeco.position import Position
import rclpy
from random import Random


class Group(ComponentRole):
    def __init__(self):
        super().__init__()
        self.center = None
        self.members = []

class Rover(ComponentRole):
    def __init__(self):
        super().__init__()
        self.position = None
        self.goal = None

class LeaderRole(ComponentRole):
    pass

# Component
class Leader(ROSComponent):
    SPEED = 0.01
    random = Random(0)
    
    @staticmethod
    def gen_random_position():
        return Position(Leader.random.uniform(0, 1), Leader.random.uniform(0, 1))

    # Knowledge definition
    class Knowledge(LeaderRole, Rover, BaseKnowledge):
        def __init__(self):
            super().__init__()

    # Component initialization
    def __init__(self, node: Node, topic):
        callback = self.PoseCallback
        super().__init__(node, Pose, topic, callback)

        # Initialize knowledge
        self.knowledge.position = node.positionProvider.get()
        self.knowledge.goal = self.gen_random_position()

    def PoseCallback(self, msg):
        self.knowledge.position.x = msg.position.x
        self.knowledge.position.y = msg.position.y

    @process(period_ms=10)
    def update_time(self, node: Node):
        self.knowledge.time = node.runtime.scheduler.get_time_ms()

    @process(period_ms=100)
    def sense_position(self, node: Node):
        self.knowledge.position = node.positionProvider.get()

    @process(period_ms=1000)
    def set_goal(self, node: Node):
        if self.knowledge.position == self.knowledge.goal:
            self.knowledge.goal = self.gen_random_position()
            node.walker.set_target(self.knowledge.goal)
        node.walker.set_target(self.knowledge.goal)


class FollowerRole(ComponentRole):
    pass

class Follower(ROSComponent):
    SPEED = 0.02
    # Knowledge definition
    class Knowledge(FollowerRole, Rover, BaseKnowledge):
        def __init__(self):
            super().__init__()

    def PoseCallback(self, msg):
        self.knowledge.position.x = msg.position.x
        self.knowledge.position.y = msg.position.y

    # Component initialization
    def __init__(self, node: Node, topic, callback=None):
        callback = self.PoseCallback
        super().__init__(node, Pose, topic, callback)

        # Initialize knowledge
        self.knowledge.position = node.positionProvider.get()
        self.knowledge.goal = None

    @process(period_ms=10)
    def update_time(self, node: Node):
        self.knowledge.time = node.runtime.scheduler.get_time_ms()

    @process(period_ms=100)
    def sense_position(self, node: Node):
        self.knowledge.position = node.positionProvider.get()

    @process(period_ms=100)
    def set_goal(self, node: Node):
        node.walker.set_target(self.knowledge.goal)

class LeaderFollowingGroup(EnsembleDefinition):
    
    class RobotGroupKnowledge(BaseKnowledge, Group):
        def __init__(self):
            super().__init__()

        def __str__(self):
            return self.__class__.__name__ + " centered at " + str(self.center) + " with component ids " + str(list(map(lambda x: x.id, self.members)))

    def __init__(self):
        super().__init__(coordinator=LeaderRole, member=FollowerRole)
 
    def fitness(self, a: Leader.Knowledge, b: Follower.Knowledge):
        return 1.0 / a.position.dist_to(b.position)

    def membership(self, a: Leader.Knowledge, b: Follower.Knowledge):
        assert isinstance(a, LeaderRole)
        assert isinstance(b, FollowerRole)
        return True

    def knowledge_exchange(self, coord: Leader.Knowledge, member: EnsembleMember[Follower.Knowledge]):
        set_goal = SetValue('goal', coord.position)
        return (coord, [set_goal])

    def __str__(self):
        return self.__class__.__name__


def test_join_ensemble_and_update_knowledge():
    rclpy.init()
    sim = ROSSim()

    # Add simple network device
    SimpleRangeLimitedNetwork(sim, range_m=3, delay_ms_mu=20, delay_ms_sigma=5)

    node0 = Node(sim)
    Walker(node0, Position(0.5, 0.5), speed_m_s=0.001/3.6)
    KnowledgePublisher(node0, publishing_period_ms=100) # same frequency that the walker
    er0 = EnsembleReactor(node0, [LeaderFollowingGroup()])
    robot0 = Leader(node0, 'robot0')
    node0.add_component(robot0)


    node1 = Node(sim)
    Walker(node1, Position(0.4, 0.6), speed_m_s=0.1/3.6)
    KnowledgePublisher(node1)
    er1 = EnsembleReactor(node1, [LeaderFollowingGroup()])
    robot1 = Follower(node1, 'robot1')
    node1.add_component(robot1)

    sim.run(10000)
    #assert has_member(er0, robot1)
    print(er0.membership)

    # check if er2 has an updated knowledge about robo
    dist = node0.positionProvider.get().dist_to(node1.positionProvider.get())
    # assert that ensemble knowledge is at most one walker step behind
    assert dist < 0.01/3.6


if __name__ == "__main__":
    test_join_ensemble_and_update_knowledge()
