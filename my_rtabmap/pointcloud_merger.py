import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudTransformer(Node):
    def __init__(self, parent_frame, child_frame):
        super().__init__("pointcloud_tf")
        self._parent_frame = parent_frame
        self._child_frame = child_frame
        self._tf_buffer = Buffer()
        self._child_to_parent_tf = TransformStamped()
        self._realsense_cloud = None
        self._livox_cloud = None
        TransformListener(self._tf_buffer, self)
        self._tf_timer = self.create_timer(0.1, self.lookup_transform)
        self._pc2_merge_timer = self.create_timer(0.1, self._merge_pointcloud)

        self._init_pubs_subs()

    def _init_pubs_subs(self):
        self.sub_realsense = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self._realsense_callback,
            10
        )

        self.sub_livox = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self._livox_callback,
            10
        )

        self.pub_combined_pc2 = self.create_publisher(
            PointCloud2, 
            '/combined_pointcloud', 
            10
        )

    def _realsense_callback(self, input_cloud: PointCloud2):
        self._realsense_cloud = input_cloud

    def _livox_callback(self, input_cloud: PointCloud2):
        self._livox_cloud = input_cloud

    def _merge_pointcloud(self):
        if self._livox_cloud is None or self._realsense_cloud is None:
            return
        
        self._realsense_cloud = do_transform_cloud(self._realsense_cloud, self._child_to_parent_tf)
        livox_points = list(pc2.read_points(self._livox_cloud, field_names=("x", "y", "z"), skip_nans=True))
        realsense_points = list(pc2.read_points(self._realsense_cloud, field_names=("x", "y", "z"), skip_nans=True))

        merged_points = livox_points + realsense_points[::10]
        # print(len(livox_points))
        # print(len(realsense_points[::10]))
        # print(len(merged_points))

        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        ]
 
        merged_header = Header()
        merged_header.stamp = self.get_clock().now().to_msg()
        merged_header.frame_id = self._livox_cloud.header.frame_id
        
        merged_cloud = pc2.create_cloud(merged_header, fields, merged_points)
        self.pub_combined_pc2.publish(merged_cloud)

    def lookup_transform(self):
        try:
            now = rclpy.time.Time()
            self._child_to_parent_tf = self._tf_buffer.lookup_transform(
                self._parent_frame, self._child_frame, now
            )
            # self.print_transform(self._child_to_parent_tf)
        except Exception as e:
            self.get_logger().error(
                f"Could not transform {self._child_frame} to {self._parent_frame}: {e}"
            )


    def print_transform(self, transform: TransformStamped):
        self.get_logger().info(
            f"Transform from {self._child_frame} to {self._parent_frame}:"
        )
        self.get_logger().info(
            f"Translation: x={round(transform.transform.translation.x, 2)}, "
            f"y={round(transform.transform.translation.y, 2)}, "
            f"z={round(transform.transform.translation.z, 2)}"
        )
        self.get_logger().info(
            f"Rotation: x={round(transform.transform.rotation.x, 2)}, "
            f"y={round(transform.transform.rotation.y, 2)}, "
            f"z={round(transform.transform.rotation.z, 2)}, "
            f"w={round(transform.transform.rotation.w, 2)}"
        )


def main(args=None):
    rclpy.init(args=args)
    parent_frame = "livox_frame"  # Replace with your source frame
    child_frame = "camera_depth_optical_frame"  # Replace with your target frame

    node = PointCloudTransformer(parent_frame, child_frame)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
