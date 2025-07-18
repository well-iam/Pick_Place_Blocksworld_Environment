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
        self.publisher_ = self.create_publisher(String, 'pyhop_plan', 100)
        
        # Debug to show the node has started
        self.get_logger().info('HTN Planner Node has started.')

        # Call PyHOP and publish
        plan = self.plan_blocks()
        self.publish_plan(plan)

    def plan_blocks(self):
        dyn = DynamicBlocksworld(self)
        state, goal = dyn.get_state_and_goal()
        return run_pyhop_plan(state, goal)

    def publish_plan(self, plan):
        if plan:
            for step in plan:
                # Convert the current plan step into a string and assign it to msg.data.
                msg = String()
                msg.data = str(step) 
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published step: {msg.data}')
        else:
            self.get_logger().error('No valid plan found.')


def main(args=None):
    rclpy.init(args=args)
    node = HTNPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()
