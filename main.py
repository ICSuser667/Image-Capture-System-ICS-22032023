#!/usr/bin/env python3
import csv
import datetime
import os
import re
import rospy
import cv2 as cv
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np
import cv2 as cv
import pandas as pd
from pypylon import pylon

filepath1 = "/home/aaeon/Desktop/StreetImages1"
filepath2 = "/home/aaeon/Desktop/StreetImages2"
latest_gps_data = ''
old_gps = ""
cap_id = 0
data_csv = []
headings = ["capture id", "lat", "lon", "filepath", "capture_date", "capture_time"]


# process the gps data
def process_gps_data(data):
    global old_gps
    global latest_gps_data
    # read latest data
    gps_data = data.data
    gps_data = str(gps_data)
    # if the gps data has been updated/changed
    if old_gps != gps_data:
        # update latest gps information
        latest_gps_data = gps_data
        old_gps = gps_data
        # split gps into lat and lon
        coord = latest_gps_data.split("#")
        rospy.loginfo("Latest gps data: %s", coord[0])
        

# save a frame from a camera
def save_frame1(data):
    br = CvBridge()
    frame = br.imgmsg_to_cv2(data)
    #print("Frame data from main:",frame)
    # testing
    # cv.imshow("image", frame)
    # cv.waitKey(1)
    # get data and time

    current_date = datetime.date.today()
    t = datetime.datetime.now()
    #print("step 1")
    time_current = str(t.hour) + ':' + str(t.minute) + ':' + str(t.second)
    global latest_gps_data
    global cap_id
    global data_csv
    coordinates = str(latest_gps_data)
    list = coordinates.split("#")
    rospy.loginfo("coordinates: %s", coordinates)

    #print("step 2")
    name = "capture_" + str(cap_id) + "_lat_" + list[0] + "_lon_" + list[1] + "_date_" + str(current_date) + ".jpeg"
    save_location = os.path.join(filepath1, name)
    cv.imwrite(save_location, frame)
    print(os.path.join(filepath1, name))
    # print("Capture " + str(cap_id) + " saved")
    data_csv.append([cap_id, list[0], list[1], save_location, current_date, time_current])
    cap_id += 1

    #img_test = cv.imread("/home/aaeon/Desktop/StreetImages1/"+name)
    #print(img_test)
    
    #return Exposure_final



# Exposure time_test
def MeanIntensity_Test(frame,Exposure,*varargin):

    # Middle value MSV is 2.5, range is 0-5
    desired_msv = 2.5
    # proportional feedback gain
    k_p = 3
    #k_p = 0.3
    LENHIST = 3
    MAX_COUNTER = 50
    counter = 0
    bComplete = False
    while not bComplete:
        counter += 1
        
        # Adjust exposure
        # Image histogram
        ImageY  = cv.cvtColor(frame, cv.COLOR_BGR2HSV)[:,:,2]
        cols = ImageY.shape[1]  #cols = 5320
        rows = ImageY.shape[0]  #rows = 3032
        hist = cv.calcHist([ImageY],[0],None,[LENHIST],[0,256])
        print("step 4") 
        # Determine mean histogram value
        mean_sample_value = np.matmul(np.linspace(1,LENHIST,LENHIST),hist)/(rows*cols)

        err_p = float(np.log2(desired_msv / mean_sample_value))
        if np.abs(err_p) > 2:
            err_p = np.sign(err_p)*2
        # print(counter,Exposure,Exposure+k_p*err_p)
        Exposure += k_p*err_p
        print("Exposure Calculating:",Exposure)
        #cap.set(cv.CAP_PROP_EXPOSURE, np.float64(Exposure))    
        if abs(err_p) < 0.2:
            bComplete = True

        if (counter > MAX_COUNTER) or (Exposure < MIN_EXPOSURE) or (Exposure > MAX_EXPOSURE):
            print('Tuning exceeded camera setting or maximum number of iteration has been exceeded.')
            break
    print('MeanIntensity iteration count = ',counter)
    #Exposure = round(Exposure,1)
    return Exposure


# Exposure time
MAX_EXPOSURE = 40000
MIN_EXPOSURE =-13

def MeanIntensity(cap,Exposure,*varargin):
    
    if len(varargin)>0:
        MAX_COUNTER = varargin[0]
    else:
        MAX_COUNTER = 50

    # Middle value MSV is 2.5, range is 0-5
    desired_msv = 5
    # proportional feedback gain
    k_p = 0.3
    LENHIST = 2.5

    counter = 0
    bComplete = False
    while not bComplete:
        counter += 1

        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
        else:   # Adjust exposure
            # Image histogram
            cols    = cap.get(cv.CAP_PROP_FRAME_WIDTH)
            rows    = cap.get(cv.CAP_PROP_FRAME_HEIGHT) 
            ImageY  = cv.cvtColor(frame, cv.COLOR_BGR2HSV)[:,:,2]            
            hist    = cv.calcHist([ImageY],[0],None,[LENHIST],[0,256])

            # Determine mean histogram value
            mean_sample_value = np.matmul(np.linspace(1,LENHIST,LENHIST),hist)/(rows*cols)

            err_p = float(np.log2(desired_msv / mean_sample_value))
            if np.abs(err_p) > 2:
                err_p = np.sign(err_p)*2
            # print(counter,Exposure,Exposure+k_p*err_p)
            Exposure += k_p*err_p
            cap.set(cv.CAP_PROP_EXPOSURE, np.float64(Exposure))    
            if abs(err_p) < 0.2:
                bComplete = True

        if (counter > MAX_COUNTER) or (Exposure < MIN_EXPOSURE) or (Exposure > MAX_EXPOSURE):
            print('Tuning exceeded camera setting or maximum number of iteration has been exceeded.')
            break
    print('MeanIntensity iteration count = ',counter)
    #Exposure = round(Exposure,1)
    return Exposure


def save_frame2(data):
    br = CvBridge()
    frame = br.imgmsg_to_cv2(data)
    # get data and time
    current_date = datetime.date.today()
    t = datetime.datetime.now()
    time_current = str(t.hour) + ':' + str(t.minute) + ':' + str(t.second)
    # testing
    # cv.imshow("image", frame)
    # cv.waitKey(1)
    global latest_gps_data
    global cap_id
    global data_csv
    coordinates = str(latest_gps_data)
    list_coord = coordinates.split("#")
    rospy.loginfo("coordinates: %s", coordinates)
    name = "capture_" + str(cap_id) + "_lat_" + list_coord[0] + "_lon_" + list_coord[1] + ".jpeg"
    save_location = os.path.join(filepath2, name)
    # cv.imwrite(save_location, frame)
    print(os.path.join(filepath2, name))
    data_csv.append([cap_id, list_coord[0], list_coord[1], save_location, current_date, time_current])
    # print("Capture " + str(cap_id) + " saved")
    cap_id += 1


def set_filepath(data):
    path = str(data)
    path = path.replace('\"', '')
    fp = re.split("#| ", path)
    print(fp)
    global filepath1
    global filepath2
    filepath1 = fp[1]
    filepath2 = fp[2]

    print("filepath 1: ", filepath1)
    print("filepath 2: ", filepath2)




def shut_down(data):
    if not data.data:
        rospy.loginfo("shutting down main node")
        global filepath1
        filepath = filepath1 + "/all_data.csv"
        with open(filepath, 'w') as f:
            # using csv.writer method from CSV package
            write = csv.writer(f)
            write.writerow(headings)
            write.writerows(data_csv)

        rospy.signal_shutdown("shutdown called ")



def run_main():
    rospy.init_node("main_node", anonymous=True)
    rospy.loginfo("main node started")
    rospy.Subscriber('filepaths', String, set_filepath)
    rospy.Subscriber('gps_coordinates', String, process_gps_data)
    rospy.Subscriber('frames', Image, save_frame1)
    rospy.Subscriber('frames2', Image, save_frame2)
    rospy.Subscriber('shutdown', Bool, shut_down)


if __name__ == '__main__':
    try:
        run_main()
        rospy.spin()
        # destroy all cv windows on completion
        cv.destroyAllWindows()
    except rospy.ROSException:
        pass