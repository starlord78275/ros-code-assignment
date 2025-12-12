#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time

class PickAndPlaceNode(Node):
    """Simple pick and place controller for UR5"""
    
    def __init__(self):
        super().__init__('pick_and_place_controller')
        
        # Publishers for joint commands
        self.joint_pubs = []
        for i in range(6):
            pub = self.create_publisher(
                Float64,
                f'/ur5/joint{i+1}_position_controller/command',
                10
            )
            self.joint_pubs.append(pub)
        
        # Subscriber for joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joints = [0.0] * 6
        self.get_logger().info("Pick and Place Node initialized")
    
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])
    
    def move_to_joints(self, target_joints, duration=2.0):
        """Move to target joint positions smoothly"""
        start_joints = self.current_joints[:]
        start_time = time.time()
        
        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            t = elapsed / duration
            
            # Linear interpolation
            current = [
                start_joints[i] + t * (target_joints[i] - start_joints[i])
                for i in range(6)
            ]
            
            # Publish joint commands
            for i, pub in enumerate(self.joint_pubs):
                msg = Float64()
                msg.data = current[i]
                pub.publish(msg)
            
            time.sleep(0.1)  # 10 Hz rate
    
    def pick_and_place_sequence(self):
        """Execute pick and place motion"""
        # Home position
        home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.get_logger().info("Moving to home position")
        self.move_to_joints(home, 2.0)
        time.sleep(1.0)
        
        # Pick position (above cube)
        pick_above = [0.5, -1.2, 1.3, -1.5, -1.57, 0.0]
        self.get_logger().info("Moving above pick position")
        self.move_to_joints(pick_above, 2.0)
        time.sleep(1.0)
        
        # Pick position (at cube)
        pick = [0.5, -1.0, 1.0, -1.4, -1.57, 0.0]
        self.get_logger().info("Picking cube")
        self.move_to_joints(pick, 1.5)
        time.sleep(1.0)
        
        # Lift cube
        self.get_logger().info("Lifting cube")
        self.move_to_joints(pick_above, 1.5)
        time.sleep(1.0)
        
        # Place position (above target)
        place_above = [1.0, -1.3, 1.4, -1.5, -1.57, 0.0]
        self.get_logger().info("Moving to place position")
        self.move_to_joints(place_above, 2.5)
        time.sleep(1.0)
        
        # Place position (at target)
        place = [1.0, -1.1, 1.1, -1.4, -1.57, 0.0]
        self.get_logger().info("Placing cube")
        self.move_to_joints(place, 1.5)
        time.sleep(1.0)
        
        # Return to home
        self.get_logger().info("Returning to home")
        self.move_to_joints(home, 2.5)
        
        self.get_logger().info("Pick and place complete!")
    
    def run(self):
        """Main execution loop"""
        time.sleep(2.0)
        self.pick_and_place_sequence()


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
