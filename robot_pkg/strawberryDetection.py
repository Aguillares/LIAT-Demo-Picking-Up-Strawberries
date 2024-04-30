from __future__ import division
import cv2
import numpy as np
# strawberryFound -> {1 : Red, 2 : Yellow, 3 : Green}

def find_contours(image):
    # Copy
    image = image.copy()
    # input, gives all the contours, contour approximation compresses horizontal,
    # vertical, and diagonal segments and leaves only their end points. For example,
    # an up-right rectangular contour is encoded with 4 points.
    # Optional output vector, containing information about the image topology.
    # It has as many elements as the number of contours.
    # we dont need it
    contours, hierarchy = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    conArea = []
    for i in range(len(contours)):
    # They should more than 200pixels squared to considered as a strawberry
        if 200 <= cv2.contourArea(contours[i]):
            conArea.append(contours[i])

    # At least the ellipse would have 5 points.
    mask = np.zeros(image.shape, np.uint8)
    cv2.drawContours(mask, conArea, -1, 255, -1)
    cv2.imshow('Mask',mask)
    
    return conArea, mask


def circle_contour(image, contour):
    # Bounding ellipse
    image_with_ellipse = image.copy()
    # easy function
    for variable in contour:
    # It should have at least 5 points to make a ellipse, if not, it won't be able to make it.
        if len(variable) <=5:
            continue
            
        ellipse = cv2.fitEllipse(variable)
        # add it
        cv2.ellipse(image_with_ellipse, ellipse, (0,0,0), 2, cv2.LINE_AA)
    
    return image_with_ellipse

def find_strawberry(image,depth_image):
    here = False
    strawberryFound = 'Nothing'
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Make a consistent size
    # get largest dimension

    image1 = image.copy()
    # we want to eliminate noise from our image. clean. smooth colors without
    # dots
    # Blurs an image using a Gaussian filter. input, kernel size, how much to filter, empty)
    image_blur = cv2.GaussianBlur(image1, (5, 5), 0)
    #image_blur = image
    # unlike RGB, HSV separates luma, or the image intensity, from
    # chroma or the color information.
    # just want to focus on color, segmentation
    image_blur_hsv = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)
   
    matrix = depth_image < 0.7 
    
    # This is for converting all to black, the ones that are greater than or equal to 0.7
    for i in range(3):
        image_blur_hsv[:,:,i] = image_blur_hsv[:,:,i]*matrix
        
    #For HSV, Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255].
    # Filter by colour
    # 0-10 hue, 
    min_redish = np.array([0, int(150), int(104)])
    max_redish = np.array([int(10/255*179), 255, 255])
    # layer
    maskredish1 = cv2.inRange(image_blur_hsv, min_redish, max_redish)
    #cv2.imwrite('maskRed1.jpg', maskRed1)
    # birghtness of a color is hue
    # 170-180 hue, modified to 150 - 160
    # minimum red amount, max red amount
    maskRedish = maskredish1
    # Intermediate Strawberries
    min_yellow1 = np.array([int(9/255*179),int(130),int(100)])
    max_yellow1 = np.array([int(27/255*179),int(255),int(255)])
    maskYellow1 = cv2.inRange(image_blur_hsv,min_yellow1,max_yellow1)

    
    maskYellow = maskYellow1 + maskYellow1
    
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
    sumMaskYellow = sum(sum(maskYellow>0))
    info =[]
    cX = 0
    cY = 0
    ind = 0
    contours = []
    cont,_=cv2.findContours(mask_bwa, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if (sumMaskBWA > 250 or sumMaskYellow) and len(cont)<4:
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
                if inside >= 0:
                    contours.append(particular)
        mask_bwa = cv2.dilate(mask_bwa,kernel, iterations=3)
    
    # Getting the center
    savedContours = []
    # w: width
    # h: height
    if strawberryFound == 'Yellow':
        here = True
        for c in contours:
            M = cv2.moments(c)
            try:
                cX=int(M["m10"]/M["m00"])
                cY=int(M["m01"]/M["m00"])
            except:
                continue
            if cX+5>640 or cY+5>480:
                distance=depth_image[cY-5:480,cX-5:640]
            else:
                distance=depth_image[cY-5:cY+5,cX-5:cX+5]
            dist1 = distance>0
            dist2 = distance<0.7
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
       
    # Red strawberries
    min_red = np.array([0, 160, 90])
    max_red = np.array([int(6/255*179), 255, 255])
    # layer
    maskRed1 = cv2.inRange(image_blur_hsv, min_red, max_red)

    # birghtness of a color is hue
    # 170-180 hue, modified to 150 - 160
    # minimum red amount, max red amount
    min_red2 = np.array([160, 160, int(0.4*255)])
    max_red2 = np.array([179, 255, 255])
    maskRed2 = cv2.inRange(image_blur_hsv, min_red2, max_red2)
    maskRed = maskRed1+maskRed2
    
    maskRed_eroded = cv2.erode(maskRed, kernel, iterations = 1)
    maskRed_dilated = cv2.dilate(maskRed_eroded,dilatedKernel, iterations=1)
    maskRed_closed = cv2.morphologyEx(maskRed_dilated, cv2.MORPH_CLOSE, kernel)
    maskRed_clean = cv2.morphologyEx(maskRed_closed,cv2.MORPH_OPEN,smallKernel)
    maskRedBool = maskRed_clean>0
    resMaskRed = sum(sum(maskRedBool))
    if resMaskRed>250:
        strawberryFound = 'Red'
        redContour, maskRed_strawberries = find_contours(maskRed)
       
    if strawberryFound == 'Red':
        here = True
        for c in redContour:
            M = cv2.moments(c)
  #          ind = ind + 1
            cX=int(M["m10"]/M["m00"])
            cY=int(M["m01"]/M["m00"])
            if cX+5>640 or cY+5>480:
                distance=depth_image[cY-5:480,cX-5:640]
            else:
                distance=depth_image[cY-5:cY+5,cX-5:cX+5]
            dist1 = distance>0
            dist2 = distance<0.7
            distanceLog=np.logical_and(dist1,dist2)    
            ind = np.where(distanceLog)
            
            avgDist = np.average(distance[ind[0],ind[1]])
            # Draw the centers
            if avgDist <=0.7:
                savedContours.append(c)
                info.append([1,cX,cY,avgDist])
                cv2.putText(image1, strawberryFound,(cX-20,c[:,0][:,1].min()-40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                cv2.putText(image1, "X = "+str(cX)+", Y = "+str(cY),(cX-30,c[:,0][:,1].min()-25),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                if avgDist>=0.3:
                    cv2.putText(image1, "D= {:.3f} m ".format(avgDist),(cX-30,c[:,0][:,1].min()-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                else:
                    cv2.putText(image1, "Close!!",(cX-30,c[:,0][:,1].min()-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                cv2.circle(image1, (cX,cY),2,(0,0,0),-1)
    
    # Green Strawberries
    #For HSV, Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255].
    # Filter by colour
    # minimum green amount, max green amount
    min_green = np.array([int(70/2), int(0.25*255), 0.15*255])
    max_green = np.array([int(100/2), int(1*255), 0.8*255])
    
    maskGreen = cv2.inRange(image_blur_hsv,min_green,max_green)
    
    maskGreen_eroded = cv2.erode(maskGreen, kernel, iterations = 1)
    maskGreen_dilated = cv2.dilate(maskGreen_eroded,dilatedKernel, iterations=1)
    maskGreen_closed = cv2.morphologyEx(maskGreen_dilated, cv2.MORPH_CLOSE, kernel)
    maskGreen_clean = cv2.morphologyEx(maskGreen_closed,cv2.MORPH_OPEN,smallKernel)

    maskGreenBool = maskGreen_clean>0
    
    resMaskGreen = sum(sum(maskGreenBool))
   
    if resMaskGreen>250:
        strawberryFound = 'Green'
        greenContour, maskGreen_strawberries = find_contours(maskGreen)
    if strawberryFound == 'Green':
        here = True
        for c in greenContour:
            M = cv2.moments(c)
            cX=int(M["m10"]/M["m00"])
            cY=int(M["m01"]/M["m00"])
            if cX+5>640 or cY+5>480:
                distance=depth_image[cY-5:480,cX-5:640]
            else:
                distance=depth_image[cY-5:cY+5,cX-5:cX+5]
            dist1 = distance>0
            dist2 = distance<0.7
            distanceLog=np.logical_and(dist1,dist2)    
            ind = np.where(distanceLog)
            avgDist = np.average(distance[ind[0],ind[1]])
            # Draw the centers
            if avgDist <=0.7:
                savedContours.append(c)
                info.append([3,cX,cY,avgDist])
                cv2.putText(image1, strawberryFound,(cX-20,c[:,0][:,1].min()-25),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                if avgDist>=0.3:
                    cv2.putText(image1, "D= {:.3f} m ".format(avgDist),(cX-30,c[:,0][:,1].min()-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                else:
                    cv2.putText(image1, "Close!!",(cX-30,c[:,0][:,1].min()-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                cv2.circle(image1, (cX,cY),2,(0,0,0),-1)
        
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