import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_map',  # Replace with your PointCloud2 topic
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Convert PointCloud2 message to a numpy array
        point_cloud = []
        for point in pc2.read_points(msg, skip_nans=True):
            point_cloud.append([point[0], point[1], point[2]])
        np_points = np.array(point_cloud)

        # Convert to Open3D PointCloud
        o3d_point_cloud = o3d.geometry.PointCloud()
        o3d_point_cloud.points = o3d.utility.Vector3dVector(np_points)

        # Save to a .ply file
        o3d.io.write_point_cloud("src/my_rtabmap/ply_data/output.ply", o3d_point_cloud)
        self.get_logger().info("PointCloud saved to output.ply")

        # Shutdown after saving the file
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver = PointCloudSaver()
    rclpy.spin(pointcloud_saver)
    pointcloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
