# VX300 FOR PICKING UP PLASTIC STRAWBERRIES USING CV FOR DETECTION

![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/vx300.png) ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/D415.png)

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
- find_strawberry().
- 
  **Part 1**

  
  
  This is the longest function of this code. It will be explained just the key parts to understand the logic of the identification.
  At the beginning there is a supposition that there are not strawberries. We make a copy of the image, we blur it to remove part of the noise. The original image comes as BGR, so we convert it into a RGB, and we convert it again but in HSV because it is easier to work with the images with this format.

  At the end we become all the pixels that have bigger distance than 0.7 m into black in the HSV image, because we want to ignore them when we detect the strawberries, since if they are not eliminated things that have the enough size to be considered as strawberry but a little far away can be detected; however, this is not desired insamuch as it can affect the detection due to things that are out of desired range (0.3 to 0.7m) .

  


      here = False
      strawberryFound = 'Nothing'
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      image1 = image.copy()
      image_blur = cv2.GaussianBlur(image, (5, 5), 0)
      image_blur_hsv = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)
      matrix = depth_image < 0.7
      for i in range(3):
            image_blur_hsv[:,:,i] = image_blur_hsv[:,:,i]*matrix

   **Part 2**

  We create the different ranges according to the colours, redish and yellow. We make several kernels depending on the task.
  We erode to remove all the small dots that can interfere with the detection. For the dilatation we have two cases one to exagerate the area of intersection and another smaller one to not deform the strawberry's shape.
  If there's intersection, it means it's a real strawberry, if there's not, it means it's not a strawberry.

  We get the sum of the number of pixels that are in the intersection. And finding the contour of all that have intersection.

  **IMPORTANT NOTE:**
  The next explanation is just for yellow-redish strawberry.

  *Case strawberry*

  WITHOUT DILATATION
  
  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/YellowDrawStrawbery.png)

  WITH DILATATION (INTERSECTION)
  
  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/IntersectionDrawStrawbery.png)

  
  *Case no strawberry*

  ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/FalseYellowStrawberry.png)
  
  
      min_redish = np.array([0, int(150), int(104)])
      max_redish = np.array([int(10/255*179), 255, 255])
      maskredish1 = cv2.inRange(image_blur_hsv, min_redish, max_redish)
      maskRedish = maskredish1
      # Intermediate Strawberries
      min_yellow1 = np.array([int(9/255*179),int(130),int(100)])
      max_yellow1 = np.array([int(27/255*179),int(255),int(255)])
      maskYellow = cv2.inRange(image_blur_hsv,min_yellow1,max_yellow1)
  
     
      
      smallKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
      dilatedKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
      kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))    
      
      maskYellow_eroded = cv2.erode(maskYellow, dilatedKernel, iterations = 1)
      maskYellow_dilatedCom = cv2.dilate(maskYellow_eroded,kernel, iterations=2)
      maskYellow_dilated = cv2.dilate(maskYellow_eroded,dilatedKernel, iterations=1)
      maskYellow_closed = cv2.morphologyEx(maskYellow_dilated, cv2.MORPH_CLOSE, kernel)
      maskYellow_clean = cv2.morphologyEx(maskYellow_closed,cv2.MORPH_OPEN,smallKernel)
      
      maskRedish_eroded = cv2.erode(maskRedish, smallKernel, iterations = 1)
      maskRedish_dilatedCom = cv2.dilate(maskRedish_eroded,kernel, iterations=2)
      maskRedish_dilated = cv2.dilate(maskRedish_eroded,dilatedKernel, iterations=1)
      maskRedish_closed = cv2.morphologyEx(maskRedish_dilated, cv2.MORPH_CLOSE, kernel)
      maskRedish_clean = cv2.morphologyEx(maskRedish_closed,cv2.MORPH_OPEN,smallKernel)
      
      
      # We can use this mask, to elimate other things, they don't have intersection
      mask_bwa = cv2.bitwise_and(maskYellow_dilatedCom,maskRedish_dilatedCom)
      sumMaskBWA = sum(sum(mask_bwa>0))
    
      info =[]
      cX = 0
      cY = 0
      ind = 0
      contours = []
      cont,_=cv2.findContours(mask_bwa, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  
    **Part 3**

  If the pixels number is greater than 250, and the strawberryFound changed to 'Yellow' now it is not 'nothing', we used the clean masks of Redish and Yellow, we find the centroids of the intersected sections but we have to check if those are within the contours we found (if it is not, it's not a real strawberry), if it is inside, it will be greater or equal to 0. 

      if sumMaskBWA > 250:
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
           
    **Part 4**
  
    If the square is crossing the limits of the screen, we are going to fix it in proportion to the maximum limit either x or y.
    
    Now that we ensure that is yellow, we are going to get the real centroid, the one is not dilated extremely. We are going to make a small square to get the average distance and to ignore the ones that less than 0.3 or greater than 0.7 (getting the index of those pixels that are in that range).
  
    We remove the area of the yellow strawberry using a black rectangule (all to zero), because if it remains there, as it has red, in the next part can detect as a red one.
    We save the information in the next order:
  
    [Type of strawberry (1,2 or 3), coordinate in X of the centroid, coordinate in Y of the centroid, Average distance]
  
    If we get more than strawberry that array will be become into a matrix, with all strawberries.
    
    To finish we put the corresponding text.
  
    It is a similar process for green and red ones. Look up the [code](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/robot_pkg/strawberryDetection.py)
   for more detail.
  
    *GRAPHIC OF DISTANCES MATRIX*
  Here you can see that the red square is indeed the centroid, while, the green ones are other pixels that are going to be calculated in the average because they are within the range [0.3, 0.7] .

    ![](https://github.com/Aguillares/LIAT-Demo-Picking-Up-Strawberries/blob/master/images/DistancesMatrix.png)
  
      #Getting the center
      savedContours = []
      #w: width
      #h: height
      if strawberryFound == 'Yellow':
          here = True
          for c in contours:
              M = cv2.moments(c)
              try:
                  cX=int(M["m10"]/M["m00"])
                  cY=int(M["m01"]/M["m00"])
              except:
                  continue
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
              # Draw the centers
              if avgDist <= 0.7:
                  savedContours.append(c)
                  x,y,w,h= cv2.boundingRect(c)
                  for i in range(3):
                       image_blur_hsv[y-10:y+h+10,x-10:x+w+10,i] = 0
                       
                  info.append([2,cX,cY,avgDist])
                  cv2.putText(image1, strawberryFound,(cX-20,c[:,0][:,1].min()-25),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                  if avgDist >=0.3:
                      cv2.putText(image1, "D= {:.3f} m ".format(avgDist),(cX-30,c[:,0][:,1].min()-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                  else:
                      cv2.putText(image1, "Close!!",(cX-30,c[:,0][:,1].min()-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                  cv2.circle(image1, (cX,cY),2,(0,0,0),-1)
  
    **Part 5**
  
  The last part it just to circle all the found contours. And we make a cross in the middle of the screen to just for ilustration. If there's nothing detected, the image is returned itself.
  
  The last three lines is for ordering in an ascending order the set of strawberries (first the red, second the yellow, third the green).

      circled1 = circle_contour(image1, savedContours)
      circled1 = cv2.line(circled1,(320,0),(320,480),color=(0,0,0),thickness=1)
      circled1 = cv2.line(circled1,(0,240),(640,240),color=(0,0,0),thickness=1)
      if here:
          bgr = cv2.cvtColor(circled1, cv2.COLOR_RGB2BGR)
      else:
          bgr = cv2.cvtColor(image1, cv2.COLOR_RGB2BGR)
      
      if len(info) == 0:
          return bgr,[[]]
      # we're done, convert back to original color scheme
      
      temp = np.array(info)
      
      info = temp[np.lexsort((temp[:,0],))]
      return bgr,info
### Camera Node
### Robot Node
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

## REFERENCES
- [Viper300X](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/vx300.html)
- [Interbotix_ros_xsarms](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)
- [Camera D415](https://www.intelrealsense.com/depth-camera-d415/)

