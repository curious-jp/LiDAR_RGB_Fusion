# publish camera parameters from yaml file
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml

class CameraParam(Node):

      def __init__(self):
          super().__init__('camera_param')
          self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info_new', 10)
          self.camera_info = CameraInfo()
          self.get_camera_info()
          self.publish_camera_info()

      def get_camera_info(self):
          with open('camera_info.yaml', 'r') as file:
               camera_info = yaml.load(file, Loader=yaml.FullLoader)
          self.camera_info.width = camera_info['image_width']
          self.camera_info.height = camera_info['image_height']
          self.camera_info.distortion_model = camera_info['distortion_model']
          self.camera_info.D = camera_info['distortion_coefficients']['data']
          self.camera_info.K = camera_info['camera_matrix']['data']
          self.camera_info.R = camera_info['rectification_matrix']['data']
          self.camera_info.P = camera_info['projection_matrix']['data']

      def publish_camera_info(self):
          while rclpy.ok():
              self.camera_info.header.stamp = self.get_clock().now().to_msg()
              self.camera_info_pub.publish(self.camera_info)
              self.get_logger().info('Publishing camera info')
              self.get_logger().info(self.camera_info)
              self.get_logger().info('---------------------')
              rclpy.spin_once(self)

def main(args=None):
  rclpy.init(args=args)
  camera_param = CameraParam()
  rate = camera_param.create_rate(50)  # Set the spin rate to 50Hz
  while rclpy.ok():
    rclpy.spin_once(camera_param)
    rate.sleep()
  camera_param.destroy_node()
  rclpy.shutdown()
