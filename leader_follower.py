from deeco.ros import ROSComponent, ROSSim
from deeco.plugins.ensemblereactor import EnsembleMember, EnsembleReactor, has_member
from deeco.core import (
    EnsembleDefinition,
    process,
    Node,
    BaseKnowledge,
    ComponentRole
)
from deeco.plugins.simplenetwork import SimpleNetwork
from deeco.plugins.knowledgepublisher import KnowledgePublisher
from deeco.mapping import SetValue
from geometry_msgs.msg import Pose
from deeco.plugins.attributes import ROSPose
from walker_pub import ROSWalker, start_walkers
import rclpy
from random import Random
from clock import fake_clock


def euclidean_distance(nodeA, nodeB):
    return ((nodeA.position.x - nodeB.position.x) ** 2 + (nodeA.position.y - nodeB.position.y) ** 2) ** 0.5

def dist_to(componentA, componentB):
    return euclidean_distance(componentA.knowledge.position, componentB.knowledge.position)


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

        self.pose_attribute = ROSPose(node, self, pose_topic)

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
    def __init__(self, node: Node, pose_topic):
        super().__init__(node)

        self.pose_attribute = ROSPose(node, self, pose_topic)

        # Initialize knowledge
        self.knowledge = Follower.Knowledge()
        self.knowledge.goal = None

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
        if b is None:
            return 0
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
    limit_ms = 1e3
    ros_frequency = 0.01
    rclpy.init()
    sim = ROSSim()

    # Add simple network device
    SimpleNetwork(sim)

    node0 = Node(sim)
    KnowledgePublisher(node0, publishing_period_ms=100) # same frequency that the walker
    er0 = EnsembleReactor(node0, [LeaderFollowingGroup()])
    leader = Leader(node0, 'leader')
    leader_pose = Pose()
    leader_pose.position.x = 0.5
    leader_pose.position.y = 0.5
    leader_walker = ROSWalker(leader, leader_pose, 'leader', speed_ms=0.05/3.6, frequency=ros_frequency)
    node0.add_component(leader)

    node1 = Node(sim)
    KnowledgePublisher(node1, publishing_period_ms=100)
    follower = Follower(node1, 'follower')
    follower_pose = Pose()
    follower_pose.position.x = 0.4
    follower_pose.position.y = 0.6
    follower_walker = ROSWalker(follower, follower_pose, 'follower', speed_ms=0.1/3.6, frequency=ros_frequency)
    node1.add_component(follower)

    fake_clock(limit=limit_ms, frequency=ros_frequency)
    start_walkers([leader_walker, follower_walker])
    sim.run(limit_ms)
    #assert has_member(er0, robot1)
    print(er0.membership)

    dist = dist_to(leader, follower)
    # assert that ensemble knowledge is at most one walker step behind
    print(dist)


if __name__ == "__main__":
    test_join_ensemble_and_update_knowledge()
