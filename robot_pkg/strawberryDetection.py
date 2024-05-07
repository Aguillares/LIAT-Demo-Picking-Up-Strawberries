# This function can be used not just for ROS2, but for a simple Python script.
# The parts that use cv2.imshow, it for the Python scripts.
from __future__ import division
import cv2
import numpy as np
# strawberryFound -> {1 : Red, 2 : Yellow, 3 : Green}

def find_contours(image):
    # Copy
    image = image.copy()
    # Input, gives all the contours, contour approximation compresses horizontal,
    # vertical, and diagonal segments and leaves only their end points. For example,
    # an up-right rectangular contour is encoded with 4 points.
    # Optional output vector, containing information about the image topology.
    # It has as many elements as the number of contours.
    contours, hierarchy = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # We are going to save all contours that pass the test of at least 200 pixels squared.
    conArea = []
    for i in range(len(contours)):
    # They should more than 200 squared pixels to considered as a strawberry
        if 200 <= cv2.contourArea(contours[i]):
            # It is added
            conArea.append(contours[i])

    # The mask is created (a zeros matrix).
    mask = np.zeros(image.shape, np.uint8)
    # The mask is modified with the contours that passed the test (at least 200 squared pixels)
    cv2.drawContours(mask, conArea, -1, 255, -1)
    
    # It is returned the contours (conArea) and the mask.
    return conArea, mask


def circle_contour(image, contour):
    # Bounding ellipse
    image_with_ellipse = image.copy()
    
    for variable in contour:
    # It should have at least 5 points to make a ellipse, if not, it won't be able to make it.
        if len(variable) <=5:
            continue
        # It makes a ellipse for that contour (variable)
        ellipse = cv2.fitEllipse(variable)
        # It is modified in image_with_ellipse
        cv2.ellipse(image_with_ellipse, ellipse, (0,0,0), 2, cv2.LINE_AA)
    
    #It is returned
    return image_with_ellipse


# This function requires the found countours, the matrix of distances, the contours that we are going to circle them
# the info to update, the image itself, the type of strawberry found and at the last the image_blur_hsv that this one
# it is used for the Yellow case since we need to eliminate that one before it will detect the red ones,
# if they are not removed they are going to be detected as red ones.
def graphic(foundContours,depth_image,savedContours,info,image1,strawberryFound,image_blur_hsv):
    # We have to identify the found strawberry found, and we are going to give different colours.
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
        # For each found countour we are going to calculate the centroids (using the moments).
    for c in foundContours:
          M = cv2.moments(c)
          cX=int(M["m10"]/M["m00"])
          cY=int(M["m01"]/M["m00"])
          # We want to make a small square around the centroid to get the average distance [(cX-5,cY-5),(upperXLim,upperYLim)]
          # if cX+5 is greater than 640, which means is outside the screen from the X Axis, we have to reduce it to 640
          if cX+5>640:
              upperXLim = 640
          else:
              upperXLim = cX+5
          # We have similar reasoning for cY+480.
          if cY+5>480:
              upperYLim = 480
          else:
              upperYLim = cY+5
          # Here it is the small the matrix (square) to get the different distances around the centroid
          distance=depth_image[cY-5:upperYLim,cX-5:upperXLim]
          
          # We make a distances range to get the desired pixels according that range [m].
          dist1 = distance>=0.3
          dist2 = distance<=0.7
          
          # We got a logical matrix
          distanceLog=np.logical_and(dist1,dist2)    
          # We got those indexes
          ind = np.where(distanceLog)
          
          # We got the average distance
          avgDist = np.average(distance[ind[0],ind[1]])
          # We are going to consider just if the distance is less or equal to 0.7 meters.
          if avgDist <=0.7:
              # It is added to savedContours
              savedContours.append(c)
              # It is found the strawberry, Yellow
              if strawberryFound == 'Yellow':
                  # We are going to use this bounding box to remove the yellow strawberry
                  x,y,w,h= cv2.boundingRect(c)
                  # We make a black rectagule to remove the strawberry
                  for i in range(3):
                       image_blur_hsv[y-10:y+h+10,x-10:x+w+10,i] = 0
              # Here we add the information [type of strawberry (Yellow,Red or Green),the distance respecting to the X axis and Y axis (screen), and the distance or Z (depth) ]
              info.append([typeStraw,cX,cY,avgDist])
              # We print the text
              cv2.putText(image1, strawberryFound,(cX-20,c[:,0][:,1].min()-40),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)
              cv2.putText(image1, "X = "+str(cX)+", Y = "+str(cY),(cX-40,c[:,0][:,1].min()-25),cv2.FONT_HERSHEY_SIMPLEX,0.35,color,2)
              #If it's greater than or equal to 0.3 m the distance is printed normally, but it says "Close!!"
              if avgDist>=0.3:
                  cv2.putText(image1, "D= {:.3f} m ".format(avgDist),(cX-30,c[:,0][:,1].min()-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)
              else:
                  cv2.putText(image1, "Close!!",(cX-30,c[:,0][:,1].min()-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)
              #We are going to make a small dot in the center according to the centroid
              cv2.circle(image1, (cX,cY),2,(0,0,0),-1)
              
              #The values are returned
    return savedContours,info,image1,image_blur_hsv
    

# This function to print the process of how to detect the strawberries
def printing(vec,names):
    for i in range(len(vec)):
        cv2.imshow(names[i],vec[i])
        #cv2.imwrite(names[i]+'.jpg',vec[i])


def find_strawberry(image,depth_image):
    
    # This is activate the printing depending of the strawberry type.
    activate = [0,0,0] # YELOW, RED, GREEN
    # Here it is used in case there is no strawberry it should return the same image 
    here = False
    # At the beginning there is no strawberry
    strawberryFound = 'Nothing'
    # The image that we detect from the camera is BGR type, so we need to convert it into RGB first
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # We copy this image
    image1 = image.copy()
    # we want to eliminate noise from our image, clean. smooth colors without dots
    # Blurs an image using a Gaussian filter (input, kernel size, how much to filter, empty)
    image_blur = cv2.GaussianBlur(image1, (5, 5), 0)
    
    # Unlike RGB, HSV separates luma, or the image intensity, from
    # chroma or the color information.
    # just want to focus on color, segmentation
    # HSV == HUE, SATURATION, VALUE
    # We change it again but now to HSV.
    image_blur_hsv = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)
    
    # In this one we make a filter of removing part of the background, all the pixels than have a greater distance than 0.7 m
    matrix = depth_image <= 0.7 
    
    # This is for converting all to black, the ones that are greater than or equal to 0.7
    for i in range(3):
        image_blur_hsv[:,:,i] = image_blur_hsv[:,:,i]*matrix
        
    #For HSV, Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255].  Brightness of a color is HUE.
    # Filter by colour
    
    # YELLOW STRAWBERRIES
    # This values can vary slightly according the enviroment, you can change them if it is needed.
    min_redish = np.array([0, int(150), int(110)])
    max_redish = np.array([int(9/255*179), 255, 255])
    
    min_redish2 = np.array([int(250/255*179), int(150), int(110)])
    max_redish2 = np.array([int(255/255*179), 255, 255])
   
    # Masks respecting the ranges.
    maskRedish1 = cv2.inRange(image_blur_hsv, min_redish, max_redish)
    maskRedish2 = cv2.inRange(image_blur_hsv, min_redish2, max_redish2)
    
    # The complete mask
    maskRedish = maskRedish1 + maskRedish2

    # minimum yellow amount, max yellow amount

    # Intermediate Strawberries
    # This values can vary slightly according the enviroment, you can change them if it is needed.
    min_yellow1 = np.array([int(9/255*179),int(125),int(125)])
    max_yellow1 = np.array([int(30/255*179),int(255),int(255)])
    maskYellow = cv2.inRange(image_blur_hsv,min_yellow1,max_yellow1)

    

    # Depending on the task we can use different kernel sizes.
    smallKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mediumKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
    bigKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (12, 11))
    
    
    # We are going to process the image
    # Removing the dots or particles that can be, that can make noise
    maskYellow_eroded = cv2.erode(maskYellow, smallKernel, iterations = 1)
    
    # We dilate the image, to amplify the strawberry to get better results for detection.
    # We elimate the small holes than can be in a strawberry image.
    # We clean image reducing the contour (open section, not inside)
    maskYellow_dilated = cv2.dilate(maskYellow_eroded,kernel, iterations=1)
    maskYellow_closed = cv2.morphologyEx(maskYellow_dilated, cv2.MORPH_CLOSE, kernel)
    maskYellow_clean = cv2.morphologyEx(maskYellow_closed,cv2.MORPH_OPEN,smallKernel)
    
    
    # These are used to detect if in deed is a Yellow strawberry, because if we exagerate the dilation the Red and Yellow part can
    # make an area that can touch each other, hence, with this we can conclude in fact it is a yellow strawberry.
    maskYellow_dilatedCom = cv2.dilate(maskYellow_eroded,mediumKernel, iterations=2)
    maskYellow_closedCom = cv2.morphologyEx(maskYellow_dilatedCom, cv2.MORPH_CLOSE, kernel)
    maskYellow_cleanCom = cv2.morphologyEx(maskYellow_closedCom, cv2.MORPH_CLOSE, kernel)
    
    
    # The next process and further have the same logic.
    maskRedish_eroded = cv2.erode(maskRedish, smallKernel, iterations = 1)
    
    maskRedish_dilated = cv2.dilate(maskRedish_eroded,kernel, iterations=1)
    maskRedish_closed = cv2.morphologyEx(maskRedish_dilated, cv2.MORPH_CLOSE, kernel)
    maskRedish_clean = cv2.morphologyEx(maskRedish_closed,cv2.MORPH_OPEN,smallKernel)
    
    
    maskRedish_dilatedCom = cv2.dilate(maskRedish_eroded,kernel, iterations=2)
    maskRedish_closedCom = cv2.morphologyEx(maskRedish_dilatedCom, cv2.MORPH_CLOSE, kernel)
    maskRedish_cleanCom = cv2.morphologyEx(maskRedish_closedCom, cv2.MORPH_CLOSE, kernel)
    
    
    # This (activate) is not used for the ROBOTIC ARM, is just to visualize the results, if you want to use it,
    # you should run it with a normal python script
    if activate[0] == 1:
        
        maskYellowVec = [maskYellow,maskYellow_eroded,maskYellow_dilated,maskYellow_closed,maskYellow_clean,maskYellow_dilatedCom,maskYellow_closedCom,maskYellow_cleanCom]
        maskYellowNames = ['maskYellow','YellowEroded','YellowDilated','YellowClosed','YellowClean','YellowDilatedCom','YellowClosedCom','YellowCleanCom']
    
        printing(maskYellowVec,maskYellowNames)
        
        maskRedishVec = [maskRedish,maskRedish_eroded,maskRedish_dilated,maskRedish_closed,maskRedish_clean,maskRedish_dilatedCom,maskRedish_closedCom,maskRedish_cleanCom]
        maskRedishNames = ['maskRedish','RedishEroded','RedishDilated','RedishClosed','RedishClean','RedishDilatedCom','RedishClosedCom','RedishCleanCom']
    
        printing(maskRedishVec,maskRedishNames)
        
        printing([maskRedish_cleanCom+maskYellow_cleanCom],['maskYellowRedishCom'])
        
        printing([maskRedish_clean+maskYellow_clean],['maskYellowRedish'])
    
    
    # We can use this mask, to elimate other things, they don't have intersection
    # With this one can see the intersection of the two masks.
    mask_bwa = cv2.bitwise_and(maskYellow_cleanCom,maskRedish_cleanCom)
    #printing([mask_bwa],["intersection"])
    # We sum the number of pixels.
    sumMaskBWA = sum(sum(mask_bwa>0))
    # We create the variables info (to know the found strawberry types), the centroids (cX,cY), the contours
    info =[]
    cX = 0
    cY = 0
    contours = []
    
    # We find the contours of the intersection mask (cont)
    cont,_=cv2.findContours(mask_bwa, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # It must be at least 200 pixels
    if sumMaskBWA >= 200:
        # We update the status, now we know it can possible Yellow strawberry
        strawberryFound = 'Yellow'
        # We get the normal and total mask of the yellow strawberry
        maskYellowRedish = maskRedish_clean + maskYellow_clean
        # We get the contours and the mask
        redishContour, maskRedish_strawberries = find_contours(maskYellowRedish)
        # We are going to evaluate if the found countours have inside of them one
        # of the found centroids of the intersected areas.
        for particular in redishContour:
            for c in cont:
                M = cv2.moments(c)
                try:
                    cX=int(M["m10"]/M["m00"])
                    cY=int(M["m01"]/M["m00"])
                except:
                    continue
                # Here it will say if it is inside
                inside = cv2.pointPolygonTest(particular,(cX,cY),False)
                # If it is inside we are surer than it can be a yellow strawberry
                if inside >=0:
                    contours.append(particular)
    # The saved contours is to circle all the strawberries
    savedContours = []
    
    # If we have a yellow strawberry, "here" to true, so we can return the changed image with the circled one.
    if strawberryFound == 'Yellow':
        here = True
        # The next line is for drawing and to get info.
        savedContours,info, image1, image_blur_hsv = graphic(contours,depth_image,savedContours,info,image1,strawberryFound,image_blur_hsv)
         
        
    #cv2.imshow('image_blur_hsv',image_blur_hsv)
    #cv2.imwrite('image_blur_hsv.jpg',image_blur_hsv)
    # RED STRAWBERRIES
    
    # IBIDEM
    min_red = np.array([0, 140, 93])
    max_red = np.array([int(6/255*179), 255, 255])
    # Layer
    maskRed1 = cv2.inRange(image_blur_hsv, min_red, max_red)

    # minimum red amount, max red amount
    min_red2 = np.array([int(240/255*179), 140, 93])
    max_red2 = np.array([int(255/255*179), 255, 255])
    maskRed2 = cv2.inRange(image_blur_hsv, min_red2, max_red2)
    
    maskRed = maskRed1+maskRed2
    
    maskRed_eroded = cv2.erode(maskRed, bigKernel, iterations = 1)
    
    maskRed_dilated = cv2.dilate(maskRed_eroded,mediumKernel, iterations=2)
    maskRed_closed = cv2.morphologyEx(maskRed_dilated, cv2.MORPH_CLOSE, kernel)
    maskRed_clean = cv2.morphologyEx(maskRed_closed,cv2.MORPH_OPEN,mediumKernel)
    
    if activate[1] == 1:
        maskRedVec = [maskRed,maskRed_eroded,maskRed_dilated,maskRed_closed,maskRed_clean]
        maskRedNames = ['maskRed','RedEroded','RedDilated','RedClosed','RedClean']
    
        printing(maskRedVec,maskRedNames)
    
    maskRedBool = maskRed_clean>0
    resMaskRed = sum(sum(maskRedBool))
    if resMaskRed>250:
        strawberryFound = 'Red'
        redContour, maskRed_strawberries = find_contours(maskRed_clean)
       
    if strawberryFound == 'Red':
        here = True
        savedContours,info, image1, _ = graphic(redContour,depth_image,savedContours,info,image1,strawberryFound,image_blur_hsv)
    
    # GREEN STRAWBERRIES
    
    # For HSV, Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255].
    # Filter by colour
    # Minimum green amount, max green amount
    min_green = np.array([int(50/255*179), int(50), 70])
    max_green = np.array([int(78/255*179), int(1*255),255])
    
    maskGreen = cv2.inRange(image_blur_hsv,min_green,max_green)
    
    maskGreen_eroded = cv2.erode(maskGreen, kernel, iterations = 1)
    
    maskGreen_dilated = cv2.dilate(maskGreen_eroded,smallKernel, iterations=2)
    maskGreen_closed = cv2.morphologyEx(maskGreen_dilated, cv2.MORPH_CLOSE, kernel)
    maskGreen_clean = cv2.morphologyEx(maskGreen_closed,cv2.MORPH_OPEN,kernel)
    
    if activate[2] == 1:
        
        maskGreenVec = [maskGreen,maskGreen_eroded,maskGreen_dilated,maskGreen_closed,maskGreen_clean]
        maskGreenNames = ['maskGreen','GreenEroded','GreenDilated','GreenClosed','GreenClean']
    
        printing(maskGreenVec,maskGreenNames)

    maskGreenBool = maskGreen_clean>0
    
    resMaskGreen = sum(sum(maskGreenBool))
   
    if resMaskGreen>250:
        strawberryFound = 'Green'
        greenContour, maskGreen_strawberries = find_contours(maskGreen_clean)
    if strawberryFound == 'Green':
        here = True
        savedContours,info, image1, _ = graphic(greenContour,depth_image,savedContours,info,image1,strawberryFound,image_blur_hsv)
        
    # Now here with the all saved contours we are going to circle them
    circled1 = circle_contour(image1, savedContours)
    # If it is true returned the modified image (circled1), but the image1.
    # And the images are converted again into BGR in any case.
    if here:
        bgr = cv2.cvtColor(circled1, cv2.COLOR_RGB2BGR)
    else:
        bgr = cv2.cvtColor(image1, cv2.COLOR_RGB2BGR)
    
    # We draw lines, vertical and horizontal to see the image center.
    bgr = cv2.line(bgr,(320,0),(320,480),color=(0,0,0),thickness=1)
    bgr = cv2.line(bgr,(0,240),(640,240),color=(0,0,0),thickness=1)
    
    # If there's no strawberries, we give just he image
    if len(info) == 0:
        return bgr,[[]]
    # Here, we arrange the info in ascending order according to the first column.
    temp = np.array(info)
    info = temp[np.lexsort((temp[:,0],))]
    # The variables are returned.
    return bgr,info
