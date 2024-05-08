# VX300 FOR PICKING UP PLASTIC STRAWBERRIES USING CV FOR DETECTION

![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/Others/vx300.png) ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/Others/D415.png)

**Robot in the Lab**
![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/Others/Robot%20in%20the%20lab.jpeg)

## INTRODUCTION
For this project it was used the robotic arm [ViperX-300](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/vx300.html), which is a robot that belongs to the Interbotix X-Series family of arms featuring the DYNAMIXEL X-Series Smart Servo Motors. ViperX-300 enables access to DYNAMIXEL Wizard Software as well as ROS. This robot has 5 degrees of freedom and a full 360 degree of rotation, the reach is 750mm, total span 1500 mm, the working payload is 750g, and it has wrist rotate.

For the detection part was used an [Intel RealSense Depth Camera D415](https://www.intelrealsense.com/depth-camera-d415/), which can be used indoor as outdoor, although, the documentation says it works within 0.5m to 3m, the minimum distance can reach is 0.3m, any distance less than this will detect as 0.0; also, it has stereoscopic depth technology, anccuray of <2% at 2m, the resolution in both (RGB and Depth) is up to 1280x720.

This experiment remained just using simple CV according to the three different types of strawberries, smiluting the different stages of ripeness: Green-YellowRedish-Red; obtaining good results. However, this could be made with the use of Machine Learning getting a better results.

A great disadvantage using CV is that the effectiveness of the code can vary depending of the quantity of light, and if something else has a similar shape or colour will detect wrongly, but for demostration purposes it works fine.

## OBJECTIVES
- To make a code to detect the three different types of strawberries: red, green, yellow-redish. 
- To make a code for the movement of the robot according to detected strawberry

## INSTALLATION
The first step that have to do is to install the software to use the robot depending on the type of architecture, that can be AMD64 (for most laptops, computers and NUCs) or ARM64 (Raspberry Pi 4B). In references you can see which robots can be used with this software (Interbotix_xsarms).
### Robotic Arm
#### Requirements
- One of the X-Series Robot Arm Kits
- Computer running Ubuntu Linux 20.04 or 22.04

#### Procedure of installation
You should open the terminal where you want it to be downloaded, where it says "humble", you should put the distrabution that you have.

| ROS Distro       | Manipulators       | Rovers             | Turrets            | Crawlers           |
| ---------------- | :----------------: | :----------------: | :----------------: | :----------------: |
| `ROS 2 Rolling`  | :x:                | :x:                | :x:                | :x:                |
| `ROS 2 Iron`     | :x:                | :x:                | :x:                | :x:                |
| `ROS 2 Humble`   | :heavy_check_mark: | :x:                | :x:                | :x:                |
| `ROS 2 Galactic` | :heavy_check_mark: | :heavy_check_mark: | :x:                | :x:                |
| `ROS 1 Noetic`   | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
| `ROS 1 Melodic`  | :x:                | :x:                | :x:                | :x:                |

Link: https://github.com/Interbotix/.github/edit/main/SECURITY.md

```
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```
#### Checks
Check that the udev rules were configured correctly and they are triggered bu the U2D2. Important!!! The U2D2 must be plugged into the USB port to show up the name as ttyDXL.

```
ls /dev | grep ttyDXL
```
#### Interbotix ROS Packages
Check that the Interbotix ROS packages were installed correctly.

```
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/interbotix_ws/install/setup.bash
ros2 pkg list | grep interbotix
```
...
interbotix_common_modules
interbotix_common_sim
interbotix_common_toolbox
interbotix_perception_modules
interbotix_perception_msgs
interbotix_perception_pipelines
interbotix_perception_toolbox
interbotix_ros_xsarms
interbotix_ros_xsarms_examples
interbotix_ros_xseries
interbotix_tf_tools
interbotix_xs_driver
interbotix_xs_modules
interbotix_xs_msgs
interbotix_xs_ros_control
interbotix_xs_rviz
interbotix_xs_sdk
interbotix_xs_toolbox
interbotix_xsarm_control
interbotix_xsarm_descriptions
interbotix_xsarm_dual
interbotix_xsarm_joy
interbotix_xsarm_moveit
interbotix_xsarm_moveit_interface
interbotix_xsarm_perception
interbotix_xsarm_ros_control
interbotix_xsarm_sim
...

If the next packages have been missed, you should check the installation process.
*interbotix_xs_sdk, interbotix_xs_msgs, interbotix_common_modules, and interbotix_xs_modules.*
#### CV libraries

We are going to build our own node of the camera. For that we need to install *pyrealsense2* module.
```
pip install pyrealsense2
```
#### Note
Clone this Git-Hub repository in the src folder of the interbotix_ws
## EXPLANATION
### Strawberry detection
- headers.
  The first thing is to import all the required modules and to classify the different strawberries, 1 for red, 2 for yellow and 3 for green.

      from __future__ import division
      import cv2
      import numpy as np
      #strawberryFound -> {1 : Red, 2 : Yellow, 3 : Green}
  
- find_contours().
  We discriminate the possible strawberries according to the area (size) in the image, because if we don't do this part, a lot of small things that have the same colour can appear as "strawberries", the return is the mask and the different contours that we considered they were strawberries.
  
      def find_contours(image):
      image = image.copy()
      contours, hierarchy = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
      conArea = []
      for i in range(len(contours)):
          if 200 <= cv2.contourArea(contours[i]):
              conArea.append(contours[i])
  
      mask = np.zeros(image.shape, np.uint8)
      cv2.drawContours(mask, conArea, -1, 255, -1)
      return conArea, mask
- circle_contour().
  This function is just to in circle the contours that were found with the previous function. We have to ensure that there are at least 5 points to make the ellipse, if there less it will result in an error.
  The resulting image is with a circled one.

      def circle_contour(image, contour):
          image_with_ellipse = image.copy()
          for variable in contour:
              if len(variable) <=5:
                  continue
              ellipse = cv2.fitEllipse(variable)
              cv2.ellipse(image_with_ellipse, ellipse, (0,0,0), 2, cv2.LINE_AA)
          return image_with_ellipse

- graphic()

  This function as it says with the name will have the main graphic part, taking decision according to the type of strawberry. We ensure they should be in the correct limits (upperXLim and upperYLim)

  If the square is crossing the limits of the screen, we are going to fix it in proportion to the maximum limit either x or y.
    
  Now that we ensure the strawberry type, we are going to get the real centroid, the one is not dilated extremely. We are going to make a small square to get the average distance and to ignore the ones that less than 0.3 or greater than 0.7 (getting the index of those pixels that are in that range).

  We remove the area if it is a yellow strawberry using a black rectangule (all to zero), because if it remains there, as it has red, in the next part can be detected as a red one.
  
  We can see that step in the next image.

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/image_blur_hsv.jpg)

  We save the information in the next order:

  [Type of strawberry (1,2 or 3), coordinate in X of the centroid, coordinate in Y of the centroid, Average distance]

  If we get more than strawberry that array will be become into a matrix, with all strawberries.
  
  To finish we put the corresponding text.
  
  Look up the [code](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/robot_pkg/strawberryDetection.py) to see more in detail the process.
  
  *GRAPHIC OF DISTANCES MATRIX*
Here you can see that the red square is indeed the centroid, while, the green ones are other pixels that are going to be calculated in the average because they are within the range [0.3, 0.7] .

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/Explanation/DistancesMatrix.png)
    
      def graphic(foundContours,depth_image,savedContours,info,image1,strawberryFound,image_blur_hsv):
          match strawberryFound:
              case 'Yellow':
                  typeStraw = 2
                  color = (0,0,0)
              case 'Red':
                  typeStraw = 1
                  color = (150,20,25)
              case 'Green':
                  typeStraw = 3
                  color = (20,200,25)
          for c in foundContours:
                M = cv2.moments(c)
                cX=int(M["m10"]/M["m00"])
                cY=int(M["m01"]/M["m00"])
               [(cX-5,cY-5),(upperXLim,upperYLim)]
                if cX+5>640:
                    upperXLim = 640
                else:
                    upperXLim = cX+5
                
                if cY+5>480:
                    upperYLim = 480
                else:
                    upperYLim = cY+5
                
                distance=depth_image[cY-5:upperYLim,cX-5:upperXLim]
                
                dist1 = distance>=0.3
                dist2 = distance<=0.7
                
                distanceLog=np.logical_and(dist1,dist2)    
                ind = np.where(distanceLog)
                avgDist = np.average(distance[ind[0],ind[1]])
  
                if avgDist <=0.7:
                    savedContours.append(c)
                    if strawberryFound == 'Yellow':
                        x,y,w,h= cv2.boundingRect(c)
                        for i in range(3):
                             image_blur_hsv[y-10:y+h+10,x-10:x+w+10,i] = 0
                    info.append([typeStraw,cX,cY,avgDist])
                    cv2.putText(image1, strawberryFound,(cX-20,c[:,0][:,1].min()-40),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)
                    cv2.putText(image1, "X = "+str(cX)+", Y = "+str(cY),(cX-40,c[:,0][:,1].min()-25),cv2.FONT_HERSHEY_SIMPLEX,0.35,color,2)
                    
                    if avgDist>=0.3:
                        cv2.putText(image1, "D= {:.3f} m ".format(avgDist),(cX-30,c[:,0][:,1].min()-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)
                    else:
                        cv2.putText(image1, "Close!!",(cX-30,c[:,0][:,1].min()-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)
                    cv2.circle(image1, (cX,cY),2,(0,0,0),-1)
          return savedContours,info,image1,image_blur_hsv
        
- find_strawberry()
  **Part 1**

  This is the longest function of this code. So, we are going to explain the main parts.
  At the beginning there is a supposition that there are no strawberries. We make a copy of the image, we blur it to remove part of the noise. The original image comes as BGR, so we convert it into a RGB, and we convert it again but in HSV because it is easier to work with the images with this format.

  At the end we become all the pixels that have bigger distance than 0.7 m into black in the HSV image, because we want to ignore them when we detect the strawberries, since if they are not eliminated things that have the enough size to be considered as strawberry but a little far away can be detected; however, this is not desired insamuch as it can affect the detection due to things that are out of desired range (0.3m to 0.7m) .

      activate = [0,0,0] # YELOW, RED, GREEN
      here = False
      strawberryFound = 'Nothing'
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      image1 = image.copy()
      image_blur = cv2.GaussianBlur(image1, (5, 5), 0)
      image_blur_hsv = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)
      matrix = depth_image <= 0.7 
      for i in range(3):
          image_blur_hsv[:,:,i] = image_blur_hsv[:,:,i]*matrix

   **Part 2**

  We create the different ranges according to the colours, redish and yellow. We make several kernels depending on the task.
  We erode to remove all the small dots that can interfere with the detection. For the dilatation we have two cases one to exagerate the area of intersection and another smaller one to not deform the strawberry's shape.
  If there's intersection, it means it's a real strawberry, if there's not, it means it's not a strawberry.

  You can see the process in the next images, to see more examples go the folder [CV Process](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process)

  **Original image**

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/RawStrawberries/Picture30.jpg)

  **Mask Yellow**

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/maskYellow.jpg)
  

   **Yellow Eroded**
  
  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/YellowEroded.jpg)


  **Yellow Dilated**
  
  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/YellowDilated.jpg)


  **Yellow Dilated Comparison**

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/YellowDilatedCom.jpg)


  **Yellow Closed**

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/YellowClosed.jpg)


  **Yellow Closed Comparison**

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/YellowClosedCom.jpg)



  **Yellow Clean**

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/YellowClean.jpg)


  **Yellow Clean Comparison**

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/YellowCleanCom.jpg)


  **Mask Yellow-Redish**

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/maskYellowRedish.jpg)


  **Mask Yellow-Redish Comparison**

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/CV%20Process/maskYellowRedishCom.jpg)

  

  We get the sum of the number of pixels that are in the intersection. And finding the contour of all that have intersection.

  **IMPORTANT NOTE:**
  The next explanation is just for yellow-redish strawberry.

  *Case strawberry*

  WITHOUT DILATATION
  
  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/Explanation/YellowDrawStrawbery.png)

  WITH DILATATION (INTERSECTION)
  
  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/Explanation/IntersectionDrawStrawbery.png)

  
  *Case no strawberry*

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/Explanation/FalseYellowStrawberry.png)
  
  
      min_redish = np.array([0, int(150), int(110)])
      max_redish = np.array([int(9/255*179), 255, 255])
      
      min_redish2 = np.array([int(250/255*179), int(150), int(110)])
      max_redish2 = np.array([int(255/255*179), 255, 255])
     
      maskRedish1 = cv2.inRange(image_blur_hsv, min_redish, max_redish)
      maskRedish2 = cv2.inRange(image_blur_hsv, min_redish2, max_redish2)
      
      maskRedish = maskRedish1 + maskRedish2
  
      min_yellow1 = np.array([int(9/255*179),int(125),int(125)])
      max_yellow1 = np.array([int(30/255*179),int(255),int(255)])
      maskYellow = cv2.inRange(image_blur_hsv,min_yellow1,max_yellow1)
    
      smallKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
      mediumKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
      kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
      bigKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (12, 11))
      
      maskYellow_eroded = cv2.erode(maskYellow, smallKernel, iterations = 1)
      
      maskYellow_dilated = cv2.dilate(maskYellow_eroded,kernel, iterations=1)
      maskYellow_closed = cv2.morphologyEx(maskYellow_dilated, cv2.MORPH_CLOSE, kernel)
      maskYellow_clean = cv2.morphologyEx(maskYellow_closed,cv2.MORPH_OPEN,smallKernel)
      
      maskYellow_dilatedCom = cv2.dilate(maskYellow_eroded,mediumKernel, iterations=2)
      maskYellow_closedCom = cv2.morphologyEx(maskYellow_dilatedCom, cv2.MORPH_CLOSE, kernel)
      maskYellow_cleanCom = cv2.morphologyEx(maskYellow_closedCom, cv2.MORPH_CLOSE, kernel)
      
      maskRedish_eroded = cv2.erode(maskRedish, smallKernel, iterations = 1)
    
      maskRedish_dilated = cv2.dilate(maskRedish_eroded,kernel, iterations=1)
      maskRedish_closed = cv2.morphologyEx(maskRedish_dilated, cv2.MORPH_CLOSE, kernel)
      maskRedish_clean = cv2.morphologyEx(maskRedish_closed,cv2.MORPH_OPEN,smallKernel)
      
      maskRedish_dilatedCom = cv2.dilate(maskRedish_eroded,kernel, iterations=2)
      maskRedish_closedCom = cv2.morphologyEx(maskRedish_dilatedCom, cv2.MORPH_CLOSE, kernel)
      maskRedish_cleanCom = cv2.morphologyEx(maskRedish_closedCom, cv2.MORPH_CLOSE, kernel)
      mask_bwa = cv2.bitwise_and(maskYellow_cleanCom,maskRedish_cleanCom)
      sumMaskBWA = sum(sum(mask_bwa>0)) 
      info =[]
      cX = 0
      cY = 0
      contours = []
      cont,_=cv2.findContours(mask_bwa, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
  
    **Part 3**

  If the pixels number is greater than 200, and the strawberryFound changed to 'Yellow' now it is not 'nothing', we used the clean masks of Redish and Yellow, we find the centroids of the intersected sections but we have to check if those are within the contours we found (if it is not, it's not a real strawberry), if it is inside, it will be greater or equal to 0.
  
  If we have a yellow strawberry, "here" to true, so we can return the changed image with the circled one.

      if sumMaskBWA > 200:
            strawberryFound = 'Yellow'
            maskYellowRedish = maskRedish_clean + maskYellow_clean
            redishContour, maskRedish_strawberries = find_contours(maskYellowRedish)
            for particular in redishContour:
                for c in cont:
                    M = cv2.moments(c)
                    try:
                        cX=int(M["m10"]/M["m00"])
                        cY=int(M["m01"]/M["m00"])
                    except:
                        continue
                    inside = cv2.pointPolygonTest(particular,(cX,cY),False)
                    if inside >=0:
                        contours.append(particular)

      savedContours = []
      if strawberryFound == 'Yellow':
          here = True
          savedContours,info, image1, image_blur_hsv = graphic(contours,depth_image,savedContours,info,image1,strawberryFound,image_blur_hsv)
  
    **Part 5**
  
  The last part it just to circle all the found contours. And we make a cross in the middle of the screen to just for ilustration. If there's nothing detected, the image is returned itself.
  
  The last three lines is for ordering in an ascending order the set of strawberries (first the red, second the yellow, third the green).

      circled1 = circle_contour(image1, savedContours)
      if here:
          bgr = cv2.cvtColor(circled1, cv2.COLOR_RGB2BGR)
      else:
          bgr = cv2.cvtColor(image1, cv2.COLOR_RGB2BGR)
      
      bgr = cv2.line(bgr,(320,0),(320,480),color=(0,0,0),thickness=1)
      bgr = cv2.line(bgr,(0,240),(640,240),color=(0,0,0),thickness=1)
      if len(info) == 0:
          return bgr,[[]]

      temp = np.array(info)
      info = temp[np.lexsort((temp[:,0],))]
      return bgr,info


  **Result image**

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/StrawberriesSets/Set30.jpg)
  
  
### Camera Node
- headers.
  The first thing is to import all the required modules.
  
      import rclpy  
      from rclpy.node import Node 
      from sensor_msgs.msg import Image  
      from std_msgs.msg import Float32MultiArray   
      from std_msgs.msg import MultiArrayDimension 
      from cv_bridge import CvBridge               
      import numpy as np     
      from robot_pkg.strawberryDetection import find_strawberry as detector  
      import pyrealsense2 as rs                                             
- Class and constructor
  We create the needed publishers, and we create the required objects to set up the camera.
      class RealSenseCameraNode(Node):
        # The constructor
        def __init__(self):
            super().__init__('realsense_camera_node')
            self.bridge = CvBridge()
            self.info_publisher = self.create_publisher(Float32MultiArray,'strawberry_info',1)
            self.color_publisher = self.create_publisher(Image, 'realsense/image', 1)
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.configure_pipeline()
            self.timer = self.create_timer(0.1, self.publish_images)
  
- configure_pipeline(self):
  Now we give the corresponding configurations.
  
        def configure_pipeline(self):
          self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
          self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
          self.device = self.pipeline_profile.get_device()
          self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
          self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
          self.pipeline.start(self.config)

- publish_images(self):
  We publish the images that we get, both the depth and the color ones. And also we send information regarding the strawberries types.
  
      def publish_images(self):
          info =[[]]
          frames = self.pipeline.wait_for_frames()
          depth_frame = frames.get_depth_frame()
          color_frame = frames.get_color_frame()
          if not depth_frame or not color_frame:
              return
          color_image = np.asanyarray(color_frame.get_data())
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
      
### Robot Node
Before going deep in the explanation, we have to be clear about the axis, the robot and camera axis, that depending on the axis the names of X, Y an Z can vary.
The next images can show the diferences and we have to be careful when we interpret the axis.

**Camera Axis**

![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/Explanation/CameraAxis.png)

**Robot Axis**

![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/Explanation/RobotAxis.png)


- headers

      import sys          
      import numpy as np  
      import math         
      import rclpy        
      from rclpy.node import Node 
      from std_msgs.msg import Float32MultiArray  
      from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS 
 
- Class and constructor
  **Important Note**
  We couldn't use the relative positions for VX300 nor VX300s with the real ones, just with the simulated ones worked fine.
  We considered to use absolute coordinates just, but still we had some problems.

  The next example explains why we couldn't put down the robot and with it to modify the robot height with respect to the strawberry height. The run code was:

      import sys
      from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
      import numpy as np
      
      
      def main():
          bot = InterbotixManipulatorXS(
              robot_model='vx300',
              group_name='arm',
              gripper_name='gripper'
          )
      
          if (bot.arm.group_info.num_joints < 5):
              bot.core.get_logger().fatal('This demo requires the robot to have at least 5 joints!')
              bot.shutdown()
              sys.exit()
              
          while(True):
              bot.arm.set_ee_pose_components(z=0.5,moving_time = 1.5)
              bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/4.0,moving_time=3)
              bot.arm.set_ee_pose_components(z=0.4, moving_time = 1)
      
          bot.shutdown()
      
      
      if __name__ == '__main__':
          main()

  You can see we go up and later we turn the robot using just the waist, however, it is not possible to descend in that direction when you give the absolute coordinates it returns to the front. Which is logical, because if you can turn the robot along the axis Z with the same absolute coordinates, so the robot is coded to get just one if that's the case (when the waist angle is 0).

  ![Robot Side View](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/videos/ROBOT%20SIDE%20VIEW.webm)
  
  ![Robot Top View](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/videos/ROBOT%20TOP%20VIEW.webm)

  One way to solve this is to put the robot a little forward; nevertheless, we have range problem now since the robot is in distance between the frame and itself of 45 cm approximately, and the minimum distance that we could test was 20cm of radio, if we gave less than that an error appears that the robot can't do that movement.

   ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/Explanation/MinimumRadioProblem.png)
  
  
  You can visit the [arm.py](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/main/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/arm.py) to see all the functins that you can use for the robot.
  We create the constructor.
  The initial Waist position will be 51º
      class Robot(Node):
        
        def __init__(self):
            super().__init__('robot')
            self.angleJoint = 51          # [degree]
            self.moving_time = 0.1
            self.deltaJoint = -1
            self.deltaStrawW = -1
            self.deltaX = 0
            self.deltaY = 0
            self.fixedZ = 0.4
            self.deltaZ = 0
            self.fixedZPixels = 95
            self.fixedZMeters = 0.085
            self.fixedZLim = 335
            self.limInfX= 340
            self.limSupX= 360

            self.bot = InterbotixManipulatorXS(
                robot_model='vx300',
                group_name='arm',
                gripper_name='gripper',
                accel_time = 0.02,
                moving_time = 3.0
            )
            self.bot
            
            if self.bot.arm.group_info.num_joints < 5:
                self.get_logger().fatal('The robot has to have at least 5 joints!')
                self.bot.shutdown()
                sys.exit()
            
            self.initialPosition()
            
            self.subscription = self.create_subscription(
                Float32MultiArray,
                'strawberry_info',
                self.callback,
                1
            )
            
            self.subscription  

## USAGE

### 1. Step. Open the Moveit.
Be sure to interbotix_ws first.
```
cd ~/interbotix_ws
```
Be careful choosing the correct robot model. The next command it's to create a twin of the robot in the program.
```
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300
```
If you want to use just simulation, you can add the part of "use_sim:=true".
```
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300 use_sim:=true
```
### 2. Step. Launch the file
If you made any modification to the nodes codes, you have to colcon build
```
colcon build --packages-select robot_pkg 
```
And later you can launch the file "robot_pkg_launch_file.launch.py"

```
ros2 launch robot_pkg robot_pkg_launch_file.launch.py
```
This last command can open the camera and to start moving the robot, and the rest the robot with camera will do their work. To detect and to go the objective when it identifies as a red strawberry.

### 3. Step finalize the process
When it's all done, you can finish all with the next command "ctrl + c"
Also you have to close the moveit simulation and to exit the terminal where that simulation was.

## DEPTH CAMERA PROBLEMS
Using D415 we got a calibration problem, even though, we could work with the project with that inconvinient. 

We got a shadow that made those pixels to be registered as closer than in fact they were.

![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/DepthCamera%20Problems/Strawberry1.png)

![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/DepthCamera%20Problems/Strawberry2.png)
## REFERENCES
- [Viper300X](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/vx300.html)
- [Interbotix_ros_xsarms](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)
- [Camera D415](https://www.intelrealsense.com/depth-camera-d415/)


