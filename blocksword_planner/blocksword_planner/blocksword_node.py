import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from . import pyhop
from .blocks_domain import DynamicBlocksworld, run_pyhop_plan

# Class defined to create a ROS 2 Node
class HTNPlannerNode(Node):
    def __init__(self):
        super().__init__('htn_planner_node')
        # Publisher that will send String messages on the topic 'pyhop_plan'
        self.plan_pub_ = self.create_publisher(String, 'pyhop_plan', 100)
        # Debug to show the node has started
        self.state_pub_ = self.create_publisher(String, 'expected_states', 100)

        self.get_logger().info('HTN Planner Node has started.')

        # Call PyHOP and publish
        result = self.plan_blocks()
        if result:
            plan, state_trace = result
            self.publish_plan(plan)
            self.publish_states(state_trace)
        else:
            self.get_logger().error('No valid plan (and states) found.')

    def plan_blocks(self):
        dyn = DynamicBlocksworld(self)
        state, goal = dyn.get_state_and_goal()
        return run_pyhop_plan(state, goal)

    def publish_plan(self, plan):
        for step in plan:
            msg = String()
            msg.data = str(step) #One message corresponds to one step of the plan
            self.plan_pub_.publish(msg)
            self.get_logger().info(f'Published plan step: {msg.data}')

    def publish_states(self, state_trace):
        for idx, st in enumerate(state_trace):
            msg = String()
            # Extract only the 'pos' attribute
            if hasattr(st, 'pos'):
                msg.data = f"{st.pos}" #One message corresponds to one State Dictionary
            else:
                msg.data = f"State {idx}: pos=undefined"

            self.state_pub_.publish(msg)
            self.get_logger().info(f'Published expected state [{idx}]: {msg.data}')
def main(args=None):
    rclpy.init(args=args)
    node = HTNPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

