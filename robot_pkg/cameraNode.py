import rclpy  # Initilize the communication
from rclpy.node import Node # To make the nodes
from sensor_msgs.msg import Image  # To read/send images
from std_msgs.msg import Float32MultiArray    # The type of array we are going to use
from std_msgs.msg import MultiArrayDimension  # To send a "matrix"
from cv_bridge import CvBridge                # To translate the images
import numpy as np                            # Numpy calculations
from robot_pkg.strawberryDetection import find_strawberry as detector  # The function created for detection
import pyrealsense2 as rs                                             # To create the camera node


# We crete the class RealSenseCameraNode
class RealSenseCameraNode(Node):
    # The constructor
    def __init__(self):
        # We initialize the node
        super().__init__('realsense_camera_node')
        # We create the object bridge to publish images
        self.bridge = CvBridge()
        # We create our publisher to the topic 'strawberry_info' of type Float32MultiArray with a queue of 1.
        self.info_publisher = self.create_publisher(Float32MultiArray,'strawberry_info',1)
        # We create our publisher to the topic 'realsense/image' of type Image with a queue of 1.
        self.color_publisher = self.create_publisher(Image, 'realsense/image', 1)
        # We are going to check if there's a connected camera
        self.pipeline = rs.pipeline()
        # We make "config" object to set the values
        self.config = rs.config()
        # We call the next function to set the correspindg values
        self.configure_pipeline()
        # We create a timer to start publishing the images to the topic each 0.1 secs.
        self.timer = self.create_timer(0.1, self.publish_images)

    def configure_pipeline(self):
        #The next two lines are required to set the camera
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        # We get the type of device (optional)
        self.device = self.pipeline_profile.get_device()
        # We set the size of depth and color images, it is better to have smaller ones because there's less information, so
        # the code can be faster, however, it shouldn't be to small or we lose data also.
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # We start with the transmission.
        self.pipeline.start(self.config)

# The next function is to publish images.
    def publish_images(self):
        # We declare info variable
        info =[[]]
        # We wait for frames.
        frames = self.pipeline.wait_for_frames()
        # We get the two frames, depth and color.
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # If there are not frames, nothing is returned
        if not depth_frame or not color_frame:
            return
        # We change the frame as an array.
        color_image = np.asanyarray(color_frame.get_data())
        # Converting to meters and as an array
        depth_image = np.asanyarray(depth_frame.get_data())/1000
        # We detect the strawberries, if there are.
        color_image, info = detector(color_image,depth_image)
        # We convert the cv2 image into imgmsg using bridge.
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        # We publish that image
        self.color_publisher.publish(color_msg)
        
        # In fact, we cannot send a matrix, so we are going to send it as an array, but
        # we are going to give the information like it were a matrix. It iwll be a simulated matrix.
        width = len(info[0])
        height = len(info)
        # Type of "Multiarray"
        mat = Float32MultiArray()
        # We append two because it is going to be a 2 dimensional array (matrix)
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim[0].label = "width"
        mat.layout.dim[1].label = "height"
        mat.layout.dim[0].size = width
        mat.layout.dim[1].size = height
        # This is going to be the total of data.
        mat.layout.dim[0].stride = width*height
        mat.layout.dim[1].stride = width
        # The offset is zero since we already put it with width.
        mat.layout.data_offset = 0
        mat.data = [float(0)]*width*height
        dstride0 = mat.layout.dim[1].stride
        offset = mat.layout.data_offset
        
        # Here you can see, that it is an array of one dimension.
        for i in range(height):
            for j in range(width):
                mat.data[offset+i*dstride0+j] = info[i][j]
        # We publish this "matrix"
        self.info_publisher.publish(mat)
   

def main(args=None):
    # We initialize communication
    rclpy.init(args=args)
    # We create the node
    node = RealSenseCameraNode()
    # We spin it
    rclpy.spin(node)
    # If there's  a stop, we destroy the node
    node.destroy_node()
    # We finish communication.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
