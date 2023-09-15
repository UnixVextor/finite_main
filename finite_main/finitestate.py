import rclpy
from rclpy.node import Node

from smach import *
from smach_ros import *
from smach_msgs.msg import *

class NavToPoint(RosState):
    
    def __init__(self,node:Node,location):
        RosState.__init__(self, node,   outcomes=['succeeded', 'aborted', 'preempted'])
        self.cli = node.create_client()
        self.node_ = node
        while not self.cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Sevice not available, wait again')
        self.req
        
    def execute(self, ud):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self.node_,self.future)
        self.result = self.future.result()
        self.node_.get_logger().info(self.result)
        return 'succeeded'


class Gripper(RosState):
    
    def __init__(self,node:Node, open:bool):
        RosState.__init__(self, node,   outcomes=['succeeded', 'aborted', 'preempted'])
        self.cli = node.create_client()
        self.node_ = node
        while not self.cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Sevice not available, wait again')
        self.req
        
    def execute(self, ud):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self.node_,self.future)
        self.result = self.future.result()
        self.node_.get_logger().info(self.result)
        return 'succeeded'


class Delay(RosState):
    
    def __init__(self,node:Node, time):
        RosState.__init__(self, node,   outcomes=['succeeded', 'aborted', 'preempted'])
        self.node_ = node
        self.rate = node.create_rate(time)
        
    def execute(self, ud):
        while rclpy.ok():
            self.node_.get_logger().info('Execute state Delay')
            self.rate.sleep()
        return 'succeeded'


def main():
    rclpy.init()
    node = rclpy.create_node('finite_state')
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    sm = StateMachine(outcomes=['--finish--'])
    with sm:
        
        StateMachine.add('NavToPoint', NavToPoint(node,), transitions={'succeeded':'BOO', 'aborted':'BOO', 'preempted':'BOO'})

    outcome = sm.execute()
    assert outcome == '--finish--'
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()