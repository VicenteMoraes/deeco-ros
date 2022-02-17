from component import Component
from ensemble import Ensemble
from deeco import Simulation
from deeco_actions.action import KnowledgeExchange
import rclpy
import time

class TestComponent(Component):
    class TestKnowledge(Component.Knowledge):
        def __init__(self):
            super(TestComponent.TestKnowledge, self).__init__()
            self.my_string = "test"

    def __init__(self, sim, name: str = "test_component"):
        super(TestComponent, self).__init__(sim, name, callback=self.execute_goal)
        self.knowledge = self.TestKnowledge()

    def execute_goal(self, goal_handle):
        if goal_handle.request.request == 'test':
            time.sleep(1)
            feedback = KnowledgeExchange.Feedback()
            feedback.feedback = self.knowledge_to_json()
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = KnowledgeExchange.Result()
        result.result = 'Success'
        return result



class TestEnsemble(Ensemble):
    def __init__(self, sim, name: str = "test_ensemble"):
        super(TestEnsemble, self).__init__(sim, name)

    def membership(self, *candidates):
        return True


if __name__ == "__main__":
    rclpy.init()
    sim = Simulation(time_limit=5)
    comp = TestComponent(sim)
    ensemble = TestEnsemble(sim)
    ensemble.add_client(comp)
    sim.start()