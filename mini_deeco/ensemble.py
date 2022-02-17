import shortuuid
import rclpy
from deeco_actions.action import KnowledgeExchange

class Ensemble:
    def __init__(self, sim, name: str = "", frequency: float = 0):
        self.frequency = frequency if frequency else sim.frequency
        self.ros_node = sim
        self.name = name
        self._uuid = shortuuid.uuid()
        self.clients = []

        self._action_client = rclpy.action.ActionClient(
            self.ros_node,
            KnowledgeExchange,
            f'ensemble_{name}'
        )

        self.ros_node.create_timer(self.frequency, self.goal_request)

    def membership(self, candidate):
        pass

    def add_client(self, *candidates):
        self.clients += [rclpy.action.ActionClient(self.ros_node, KnowledgeExchange, candidate.server_name)
                         for candidate in candidates if self.membership(candidate)]

    def goal_request(self):
        for client in self.clients:
            msg = KnowledgeExchange.Goal()
            msg.request = "test"
            client.wait_for_server()
            goal_future =  client.send_goal_async(msg, feedback_callback=self.feedback_callback)
            goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal Rejected')
        else:
            print("Goal Accepted")
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        print(f'result: {result}')

    def feedback_callback(self, feedback):
        feedback_msg = feedback.feedback
        print(f'feedback_msg: {feedback_msg}')