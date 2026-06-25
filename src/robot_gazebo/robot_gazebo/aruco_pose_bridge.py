import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from aruco_opencv_msgs.msg import ArucoDetection
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class ArucoPoseBridge(Node):
    def __init__(self):
        super().__init__('aruco_pose_bridge')

        self.declare_parameter('target_marker_id', 0)
        self.target_id = int(self.get_parameter('target_marker_id').value)
        self.declare_parameter('target_frame', 'odom')
        self.target_frame = str(self.get_parameter('target_frame').value)  # ← cast to str

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.cb,
            10
        )
        self.pub = self.create_publisher(
            PoseStamped,
            '/detected_dock_pose',
            10
        )
        self.get_logger().info(f'Bridge ready — watching marker ID {self.target_id}')

    def cb(self, msg: ArucoDetection):
        for marker in msg.markers:
            if marker.marker_id == self.target_id:
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose = marker.pose

                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        msg.header.frame_id,
                        msg.header.stamp
                    )
                    # ← use tf2_geometry_msgs.do_transform_pose which accepts PoseStamped
                    pose_transformed = tf2_geometry_msgs.do_transform_pose_stamped(ps, transform)
                    pose_transformed.header.stamp = self.get_clock().now().to_msg()
                    self.pub.publish(pose_transformed)
                except Exception as e:
                    self.get_logger().warn(f'TF transform failed: {e}')
                return


def main():
    rclpy.init()
    node = ArucoPoseBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()