#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleAttitude
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class AttitudeVisualizer(Node):
    def __init__(self):
        super().__init__('attitude_visualizer')
        
        # Configure QoS profile to match PX4's BEST_EFFORT QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to PX4 attitude topic with BEST_EFFORT QoS
        self.subscription = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
            
        # Publisher for RViz Pose
        self.pose_pub = self.create_publisher(PoseStamped, '/px4_visual/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('🚀 PX4 Attitude Visualizer Node Started! Listening to /fmu/out/vehicle_attitude...')

    def attitude_callback(self, msg):
        # PX4 VehicleAttitude q = [w, x, y, z] in FRD/NED frame
        q_w = float(msg.q[0])
        q_x = float(msg.q[1])
        q_y = float(msg.q[2])
        q_z = float(msg.q[3])
        
        # Standard ROS 2 ENU (East-North-Up) / FLU (Forward-Left-Up) frame mapping:
        # X axis (Red)   = Forward (Roll rotation)
        # Y axis (Green) = Left (Pitch rotation)
        # Z axis (Blue)  = Up (Yaw rotation)
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        pose_msg.pose.orientation.x = q_x
        pose_msg.pose.orientation.y = -q_y
        pose_msg.pose.orientation.z = -q_z
        pose_msg.pose.orientation.w = q_w
        
        self.pose_pub.publish(pose_msg)
        
        # Publish TF transform from 'world' to 'base_link'
        t = TransformStamped()
        t.header.stamp = pose_msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = AttitudeVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
