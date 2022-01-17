from deeco.ros import ROSComponent, ROSSim
from deeco.plugins.ensemblereactor import EnsembleMember, EnsembleReactor, has_member
from deeco.core import (
    EnsembleDefinition,
    process,
    Node,
    BaseKnowledge,
    ComponentRole
)
from deeco.plugins.simplenetwork import SimpleNetwork, SimpleRangeLimitedNetwork
from deeco.plugins.knowledgepublisher import KnowledgePublisher
from deeco.mapping import SetValue
from geometry_msgs.msg import Pose
from deeco.plugins.attributes import ROSPose
from walker_pub import ROSWalker, start_walkers
import rclpy
from random import Random
from clock import fake_clock


def euclidean_distance(nodeA, nodeB):
    return ((nodeA.x - nodeB.x) ** 2 + (nodeA.y - nodeB.y) ** 2) ** 0.5


def dist_to(componentA, componentB):
    return euclidean_distance(componentA.knowledge.position.position, componentB.knowledge.position.position)

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
        pose = Pose()
        pose.position.x = Leader.random.uniform(0, 1)
        pose.position.y = Leader.random.uniform(0, 1)
        return pose

    # Knowledge definition
    class Knowledge(LeaderRole, Rover, BaseKnowledge):
        def __init__(self):
            super().__init__()
            self.position = Pose()

    # Component initialization
    def __init__(self, node: Node, pose_topic="pose"):
        super().__init__(node)

        pose_attribute = ROSPose(node, self, pose_topic)
        super().add_attribute(pose_attribute)

        # Initialize knowledge
        self.knowledge = Leader.Knowledge()
        self.knowledge.goal = self.gen_random_position()

    @process(period_ms=10)
    def update_time(self, node: Node):
        self.knowledge.time = node.runtime.scheduler.get_time_ms()

    @process(period_ms=1000)
    def set_goal(self, node: Node):
        if self.knowledge.position.position.x == self.knowledge.goal.position.x \
                and self.knowledge.position.position.y == self.knowledge.goal.position.y:
            self.knowledge.goal = self.gen_random_position()


class FollowerRole(ComponentRole):
    pass


class Follower(ROSComponent):
    SPEED = 0.02

    # Knowledge definition
    class Knowledge(FollowerRole, Rover, BaseKnowledge):
        def __init__(self):
            super().__init__()
            self.position = Pose()

    # Component initialization
    def __init__(self, node: Node, pose_topic, leader_topic):
        super().__init__(node)

        pose_attribute = ROSPose(node, self, pose_topic)
        super().add_attribute(pose_attribute)

        # Initialize knowledge
        self.knowledge = Follower.Knowledge()
        self.knowledge.goal = None

        # Set goal
        self.ros_sub = self.ros_node.create_subscription(Pose, leader_topic, self.set_goal, 10)

    @process(period_ms=10)
    def update_time(self, node: Node):
        self.knowledge.time = node.runtime.scheduler.get_time_ms()

    def set_goal(self, goal):
        self.knowledge.goal = goal


class LeaderFollowingGroup(EnsembleDefinition):
    
    class RobotGroupKnowledge(BaseKnowledge, Group):
        def __init__(self):
            super().__init__()

        def __str__(self):
            return self.__class__.__name__ + " centered at " + str(self.center) + " with component ids " + str(list(map(lambda x: x.id, self.members)))

    def __init__(self):
        super().__init__(coordinator=LeaderRole, member=FollowerRole)
 
    def fitness(self, a: Leader.Knowledge, b: Follower.Knowledge):
        return 1.0 / euclidean_distance(a.position, b.position)

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
    KnowledgePublisher(node0, publishing_period_ms=100) # same frequency that the walker
    er0 = EnsembleReactor(node0, [LeaderFollowingGroup()])
    leader = Leader(node0, 'leader')
    leader_pose = Pose()
    leader_pose.position.x = 0.5
    leader_pose.position.y = 0.5
    leader_walker = ROSWalker(leader, leader_pose, 'leader', speed_ms=0.001/3.6)
    node0.add_component(leader)

    node1 = Node(sim)
    KnowledgePublisher(node1)
    er1 = EnsembleReactor(node1, [LeaderFollowingGroup()])
    follower = Follower(node1, 'follower', leader_topic='leader')
    follower_pose = Pose()
    follower_pose.position.x = 0.4
    follower_pose.position.y = 0.6
    follower_walker = ROSWalker(follower, follower_pose, 'follower', speed_ms=0.1/3.6)
    node1.add_component(follower)

    fake_clock()
    start_walkers([leader_walker, follower_walker])
    sim.run(10000)
    #assert has_member(er0, robot1)
    print(er0.membership)

    # check if er2 has an updated knowledge about robo
    dist = dist_to(leader, follower)
    # assert that ensemble knowledge is at most one walker step behind
    print(dist)


if __name__ == "__main__":
    test_join_ensemble_and_update_knowledge()
