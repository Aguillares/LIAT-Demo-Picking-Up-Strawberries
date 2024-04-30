import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from cv_bridge import CvBridge
import numpy as np
from robot_pkg.strawberryDetection import find_strawberry as detector  # Assuming this import is correct
import pyrealsense2 as rs

class RealSenseCameraNode(Node):
    def __init__(self):
        super().__init__('realsense_camera_node')
        self.bridge = CvBridge()
        self.info_publisher = self.create_publisher(Float32MultiArray,'strawberry_info',1)
        self.color_publisher = self.create_publisher(Image, 'realsense/image', 1)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.configure_pipeline()
        self.timer = self.create_timer(0.1, self.publish_images)

    def configure_pipeline(self):
        
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

    def publish_images(self):
        info =[[]]
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return
        color_image = np.asanyarray(color_frame.get_data())
        # Converting to meters
        depth_image = np.asanyarray(depth_frame.get_data())/1000
        color_image, info = detector(color_image,depth_image)
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.color_publisher.publish(color_msg)
        
        width = len(info[0])
        height = len(info)
        mat = Float32MultiArray()
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim[0].label = "width"
        mat.layout.dim[1].label = "height"
        mat.layout.dim[0].size = width
        mat.layout.dim[1].size = height
        mat.layout.dim[0].stride = width*height
        mat.layout.dim[1].stride = width
        mat.layout.data_offset = 0
        mat.data = [float(0)]*width*height
        
        dstride0 = mat.layout.dim[1].stride
        offset = mat.layout.data_offset
        
        for i in range(height):
            for j in range(width):
                mat.data[offset+i*dstride0+j] = info[i][j]
                
        self.info_publisher.publish(mat)
   

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
