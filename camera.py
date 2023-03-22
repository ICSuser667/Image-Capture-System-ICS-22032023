#!/usr/bin/env python3
import time
import rospy
import cv2 as cv
import numpy as np
from pypylon import pylon
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import main

cameraID = 1
exposure_val = 8000

# set converter to opencv bgr format
converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    

# main capture loop
def start_camera(data):
    global exposure_val
    pub = rospy.Publisher('frames', Image, queue_size=10)

    # find all connected cameras
    tlFactory = pylon.TlFactory.GetInstance()
    devices = tlFactory.EnumerateDevices()

    # connecting to the camera with cameraID
    if len(devices)>=cameraID:

        # camera object
        camera = pylon.InstantCamera()
        camera.Attach(tlFactory.CreateDevice(devices[cameraID-1]))
        camera.Open()

        #img = cv.VideoCapture(0,cv.CAP_DSHOW)
        #img = cv.VideoCapture(1000)

        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)        # create a video stream by continually grabbing the latest image
        camera.AutoFunctionROIUseBrightness.SetValue(True)
        
        ############

        camera.ExposureAuto.SetValue("Once")     

        #camera.ExposureAuto.SetValue("Continuous")
        #camera.AutoTargetBrightness.SetValue(0.6)      # Brightness control, based on average gray value, range from 0 to 1
        camera.Gamma.SetValue(1)                       # Gamma control, range from 0 to 4

        ##########

        # create a image converter and set converter to opencv bgr format
        #converter = pylon.ImageFormatConverter()
        #converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        #converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        
        br = CvBridge()
        
        # setup original Exposure
        Exposure = 20000

        # while the node is running
        while not rospy.is_shutdown():

            ##
            grabResult = camera.RetrieveResult(500, pylon.TimeoutHandling_ThrowException)
            # if the capture has happened properly
            if grabResult.GrabSucceeded():
                # Access the image data and turn into a usable result
                image_converted = converter.Convert(grabResult)
                frame = image_converted.GetArray()
            
            ############################################
            # Exposure time

            # Using No.1 MeanIntensity
                #Exposure = MeanIntensity_Test(frame,Exposure)
            # Using No.2 GradientScore
                Exposure = GradientScore(frame,Exposure)
            # Using No.3 GradientScore_SatuMask     
                #Exposure = GradientScore_SatuMask(frame,Exposure)
                
                #print("Final Exposure", Exposure)
                camera.ExposureTime.SetValue(Exposure)
                ###############################################
                # publish the frame
                pub.publish(br.cv2_to_imgmsg(frame))
                # uncomment to show camera feed
                show_image(frame)
                # set the exposure for the next capture
                grabResult.Release()
            else:
                print("capture was false")
        #Exposure1 = main.save_frame1(data)
        #rospy.loginfo("Exposure time: %s",Exposure1)
        camera.StopGrabbing()
        camera.Close()

    else:
        print('Can''t find a matching camera number')



# Exposure time_test
# value for a2A5320
MAX_EXPOSURE = 10000000
MIN_EXPOSURE = 16       
def MeanIntensity_Test(frame,Exposure,*varargin):

    # Middle value MSV is 2, range is 1-3
    desired_msv = 2
    # proportional feedback gain
    k_p = 100
    #k_p = 0.3
    LENHIST = 3
    #LENHIST = 3
    MAX_COUNTER = 10
    #MAX_COUNTER = 50
    counter = 0
    bComplete = False
    while not bComplete:
        counter += 1
        
        # Adjust exposure
        # Image histogram
        ImageY  = cv.cvtColor(frame, cv.COLOR_BGR2HSV)[:,:,2]
        #print(ImageY)
        cols = ImageY.shape[1]  #cols = 5320
        rows = ImageY.shape[0]  #rows = 3032
        hist = cv.calcHist([ImageY],[0],None,[LENHIST],[0,256])
        print("step Mean1") 
        # Determine mean histogram value
        mean_sample_value = np.matmul(np.linspace(1,LENHIST,LENHIST),hist)/(rows*cols)
        print(mean_sample_value) 
        err_p = float(np.log2(desired_msv / mean_sample_value))
        if np.abs(err_p) > 2:
            err_p = np.sign(err_p)*2
            print(counter,Exposure,Exposure+k_p*err_p)
        Exposure += k_p*err_p
        print("Error",err_p)
        print("Exposure Calculating (MeanIntensity):",Exposure)
        #cap.set(cv.CAP_PROP_EXPOSURE, np.float64(Exposure))    
        if abs(err_p) < 0.2:
            bComplete = True
        if (counter > MAX_COUNTER) or (Exposure < MIN_EXPOSURE) or (Exposure > MAX_EXPOSURE):
            print('Tuning exceeded camera setting or maximum number of iteration has been exceeded.')
            break
    print('MeanIntensity iteration count = ',counter)
    #Exposure = round(Exposure,1)
    return Exposure

# NO.2 GradientScore
def GradientScore(frame,Exposure,*varargin):
    
    if len(varargin)>0:
        MAX_COUNTER = varargin[0]
    else:
        MAX_COUNTER = 0
    
    delta   = 0.05      #between 0.01-0.1
    lambd   = 1e3
    Kp      = 800
    edgewidth = 5

    # backup
    #delta   = 0.04
    #lambd   = 5e2
    #Kp      = 50
    #edgewidth = 3

    compress = 16

    #gamma = np.array([0.7, 1.00, 1.30])
    gamma = np.array([0.50, 0.67, 0.85, 1.00, 1.20, 1.50, 2.00])

    lengamma = len(gamma)
    cols = int(5320/compress)
    rows = int(3032/compress)
    dim = (cols,rows)
    frame = cv.resize(frame, dim, interpolation=cv.INTER_AREA)

    bContinue = True
    LoopCount = 0
    while bContinue:
        LoopCount += 1

        ImageY = cv.cvtColor(frame, cv.COLOR_BGR2HSV)[:,:,2]
        #cv.imshow('image',ImageY)
        
        #cols = ImageY.shape[1]  #cols = 5320
        #rows = ImageY.shape[0]  #rows = 3032

        m = np.zeros(lengamma)
        for idx, g in enumerate(gamma): # Higher g put more weights on stronger edges
            Y = np.power(ImageY,g)
            sobelx  = cv.Sobel(Y,cv.CV_64F,1,0,ksize=edgewidth)
            sobely  = cv.Sobel(Y,cv.CV_64F,0,1,ksize=edgewidth)
            GradSq    = np.array(sobelx*sobelx+sobely*sobely)
            ImageGrad = GradSq/np.amax(GradSq)
                # cv.imshow('gradient',ImageGrad)
                # cv.waitKey(0)

            bDenoise = ImageGrad > delta
            m[idx]  = np.sum(np.log10(lambd*(ImageGrad[bDenoise]-delta)+1))
            m[idx] /= np.log10(lambd*(1-delta)+1)
            # print(m)
            
        ptbest = np.argmax(m)
        logdE  =  Kp*NonlinearGain(gamma[ptbest])
        print("Adjust Value/Max Value:",logdE,"/",Kp)
        # print(LoopCount,Exposure,Exposure+logdE)
        Exposure += logdE
        print("Realtime Exposure (GradientScore):",Exposure)

        #cap.set(cv.CAP_PROP_EXPOSURE, np.float64(Exposure))
        if np.abs(logdE) < 10:
            bContinue = False
                
        if (LoopCount > MAX_COUNTER) or (Exposure < MIN_EXPOSURE) or (Exposure > MAX_EXPOSURE):
            #print('Tuning exceeded camera setting or maximum number of iteration has been exceeded.')
            break
    #print('GradientScore iteration count = ',LoopCount)
    return Exposure

# No.3 GradientScore_SatuMask
def GradientScore_SatuMask(frame,Exposure,*varargin):
    
    if len(varargin)>0:
        MAX_COUNTER = varargin[0]
    else:
        MAX_COUNTER = 1
    
    delta   = 0.04
    lambd   = 1e3
    Kp      = 100
    #Kp      = 0.3
    edgewidth = 3

    gamma = [0.50, 0.67, 0.85, 1.00, 1.20, 1.50, 2.00]
    lengamma = len(gamma)

    bContinue = True
    LoopCount = 0
    while bContinue:
        LoopCount += 1
        ret = True
        ImageY = cv.cvtColor(frame, cv.COLOR_BGR2HSV)[:,:,2]
        print("Step Gradient1")
        #print(ImageY)
        cols = ImageY.shape[1]  #cols = 5320
        rows = ImageY.shape[0]  #rows = 3032
            
        ret,bSatuMap = cv.threshold(ImageY, 250, 255, cv.THRESH_BINARY)
        bSatuMap = cv.dilate(bSatuMap,cv.getStructuringElement(cv.MORPH_RECT,[edgewidth,edgewidth]))
        cv.imshow('saturation',bSatuMap)
        
        m = np.zeros(lengamma)
        #m1 = np.zeros(lengamma)
        for idx, g in enumerate(gamma): # Higher g -> darker  

            Y = np.power(ImageY,g)
            sobelx  = cv.Sobel(Y,cv.CV_64F,1,0,ksize=edgewidth)
            sobely  = cv.Sobel(Y,cv.CV_64F,0,1,ksize=edgewidth)
            GradSq    = np.array(sobelx*sobelx+sobely*sobely)
            ImageGrad = GradSq/np.amax(GradSq)
            # cv.imshow('gradient',ImageGrad)
            ImageGrad[bSatuMap==255] *= 0.25
            # cv.imshow('weightedgradient',ImageGrad)
            # cv.waitKey(0)

            bDenoise = ImageGrad > delta
            m[idx]  = np.sum(np.log10(lambd*(ImageGrad[bDenoise]-delta)+1))
            m[idx] /= np.log10(lambd*(1-delta)+1)
            # print(m)
        print("Step Gradient2")
        ptbest = np.argmax(m)
        logdE  =  Kp*NonlinearGain(gamma[ptbest])
        # print(LoopCount,Exposure,Exposure+logdE)
        Exposure += logdE
        print("Exposure Calculating (GradientScore):",Exposure)
        #cap.set(cv.CAP_PROP_EXPOSURE, np.float64(Exposure))
        if np.abs(logdE) < 0.2:
            bContinue = False
                
        if (LoopCount > MAX_COUNTER) or (Exposure < MIN_EXPOSURE) or (Exposure > MAX_EXPOSURE):
            print('Tuning exceeded camera setting or maximum number of iteration has been exceeded.')
            break
    print('GradientScore_SatuMask iteration count = ',LoopCount)
    return Exposure

def NonlinearGain(g):
    # Nonlinear function controls the feedback gain. Simpler and faster than the ad-hoc gain adjustment presented in Shim et. al.(2018)
    p = 1.5
    q = 2
    if g < 0.5:
        R = 2        
    elif g < 1:
        R = 1+(2*(1-g))**p
    elif g < 2:
        R = 1-0.5*(1-g)**q
    elif g >= 2:
        R = 0.5
    R = np.log2(R) 
    #print("Test")
    #print(R)   
    return R


# method to show image
def show_image(img):
    cv.namedWindow('Image Window 1', cv.WINDOW_NORMAL)
    cv.imshow("Image Window 1", img)
    cv.waitKey(3)

# shutdown house cleaning
def shut_down(data):
    if not data.data:
        rospy.signal_shutdown("shutdown called ")


def main1():
	rospy.init_node('camera_node', anonymous=True)
	rospy.loginfo("Camera node 1 started")
	rospy.Subscriber('shutdown', Bool, shut_down)
	#rospy.Subscriber('start', Bool, start_camera)
	start_camera(True)


if __name__ == '__main__':
    try:
        main1()
        #rospy.spin()
		
    except rospy.ROSException:
        print(rospy.ROSException)
