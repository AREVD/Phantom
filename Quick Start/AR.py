
from ast import Try
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import concurrent.futures
from PyQt5.QtWidgets import QLabel, QHBoxLayout, QVBoxLayout, QApplication, QWidget
from picamera2 import Picamera2
from PyQt5.QtGui import QImage,QPixmap
from PyQt5.QtCore import QThread
import RPi.GPIO as gp
import time
import os
import cv2 as cv
import numpy as np
from scipy import linalg
from scipy.spatial import distance
import matplotlib.pyplot as plt
import keyboard

width = 320
height = 240 

adapter_info = {  
    "A" : {   
        "i2c_cmd":"i2cset -y 10 0x70 0x00 0x04",
        "gpio_sta":[0,0,1],
    }, "C" : {
        "i2c_cmd":"i2cset -y 10 0x70 0x00 0x06",
        "gpio_sta":[0,1,0],
    }
}

def select_channel(index):
        channel_info = adapter_info.get(index)
        if channel_info == None:
            print("Can't get this info")
        gpio_sta = channel_info["gpio_sta"] # gpio write
        gp.output(7, gpio_sta[0])
        gp.output(11, gpio_sta[1])
        gp.output(12, gpio_sta[2])

def init_i2c(index):
    channel_info = adapter_info.get(index)
    os.system(channel_info["i2c_cmd"]) # i2c write

def triangulate(mtx1, mtx2, R, T, ptA, ptB):
    pointA = [[159,152]]                # camA location of target
    pointA.append(ptA)
    pointB = [[111,206]]                # camB location of target
    pointB.append(ptB)
    pointA = np.array(pointA)
    pointB = np.array(pointB)

    RT1 = np.concatenate([np.eye(3), [[0], [0], [0]]], axis=-1)
    P1 = mtx1 @ RT1  # projection matrix for C1

    # RT matrix for C2 is the R and T obtained from stereo calibration.
    RT2 = np.concatenate([R, T], axis=-1)
    P2 = mtx2 @ RT2  # projection matrix for C2

    def DLT(P1, P2, point1, point2):

        A = [point1[1] * P1[2, :] - P1[1, :],
             P1[0, :] - point1[0] * P1[2, :],
             point2[1] * P2[2, :] - P2[1, :],
             P2[0, :] - point2[0] * P2[2, :]
             ]
        A = np.array(A).reshape((4, 4))
        # print('A: ')
        # print(A)

        B = A.transpose() @ A
        U, s, Vh = linalg.svd(B, full_matrices=False)

        #print('Triangulated point: ')
        #print(Vh[3, 0:3] / Vh[3, 3])
        return Vh[3, 0:3] / Vh[3, 3]

    p3ds = []
    for pointA, pointB in zip(pointA, pointB):
        _p3d = DLT(P1, P2, pointA, pointB)
        p3ds.append(_p3d)
    p3ds = np.array(p3ds)
    
    dst = distance.euclidean(p3ds[0],p3ds[1])

    return dst
    
def increase_img_brightness(img, level):
    hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
    h,s,v = cv.split(hsv)
    v += level
    final_hsv = cv.merge((h,s,v))
    img = cv.cvtColor(final_hsv, cv.COLOR_HSV2BGR)
    return img

if __name__ == "__main__":
    c = ''

    gp.setmode(gp.BOARD)
    gp.setup(32,gp.OUT)

    p = gp.PWM(32,25)
    p.start(25)

    gp.setwarnings(False)
    gp.setmode(gp.BOARD)
    gp.setup(7, gp.OUT)
    gp.setup(11, gp.OUT)
    gp.setup(12, gp.OUT)

    flag = False

    mtxA = np.load('mtxA.npy', allow_pickle=True)
    mtxB = np.load('mtxB.npy', allow_pickle=True)
    R = np.load('R.npy', allow_pickle=True)
    T = np.load('T.npy', allow_pickle=True)

    for item in {"A", "C"}:
        try:
            select_channel(item)
            init_i2c(item)
            if flag == False:
                flag = True
            else:
                picam2.close()
            picam2 = Picamera2()
            picam2.configure(picam2.create_still_configuration(
                main={"size": (width, height), "format": "RGB888"}))  # ,buffer_count=2))
            picam2.start()
            time.sleep(2)
            picam2.capture_array(wait=False)
            time.sleep(0.1)
        except Exception as e:
            print("except: " + str(e))

    while True:
        ptA = []  # initialize the pts as list
        ptC = []  # (correct input type for triangulation function)
        for item in ["A", "C"]:
            select_channel(item)  # switch to correct camera channel
            time.sleep(0.02)
            buf = picam2.capture_array()  # captures frame
            if item == "A":
                cbuf = buf[height // 5:7 * height // 8, 138 * width // 320:4 * width // 5]  # crops image

            if item == "C":
                cbuf = buf[2 * height // 7:height, 0:4 * width // 5]  # crops image
                cbuf = increase_img_brightness(cbuf, 255)

            if buf is not None:
                if item == "A":
                    lower = [14, 110, 0]  # color green boundaries
                    upper = [70, 255, 156]
                elif item == "C":
                    lower = [14, 140, 0]  # color green boundaries
                    upper = [70, 255, 156]
                lower = np.array(lower, dtype="uint8")  # make into np arrays
                upper = np.array(upper, dtype="uint8")
                mask = cv.inRange(cbuf, lower, upper)  # creating the mask for catheter tip
                points = np.where(mask == [255])  # detect all points around tip of catheter
                avg = []
                if points[0].any():
                    avg = [sum(x) // len(x) for x in list(points)]  # averages all detected points
                    # avg = [avg[1],avg[0]]
                    if item == "A":  # set ptA to avg point
                        avg = [avg[1] + (138 * width // 320),
                               avg[0] + (height // 5)]  # transforms point back to original coordinate planes
                        ptA = avg
                    if item == "C":  # set ptC to avg point
                        avg = [avg[1],
                               avg[0] + (2 * height // 7)]  # transforms point back to original coordinate planes
                        ptC = avg
                # else:
                #    continue
                if avg:  # draw red dot at detected point
                    buf = cv.circle(buf, tuple(avg), radius=3, color=(0, 0, 255), thickness=-1)
                if item == "A":  # show camera A frame
                    cv.imshow("Camera A", buf)
                if item == "C":  # show camera C frame
                    cv.imshow("Camera C", buf)
                if cv.waitKey(1) == ord('q'):
                    break

            else:
                print('failed: buf %s is None' % item)

        if ptA and ptC:  # if point detected in both cameras, triangulate
            c = str('%.2f' % (abs(6.6 * round(triangulate(mtxA, mtxB, R, T, ptA, ptC), 2) - 1.7)))
            print('distance to target: %s mm' % c)
        # elif not ptA and not ptC:
        #    print('TRIANGULATION FAILED: no points detected in camera A or camera C')
        # elif not ptA:
        #    print('TRIANGULATION FAILED: no point detected in camera A')
        # elif not ptC:
        #    print('TRIANGULATION FAILED: no point detected in camera C')

    picam2.close()


    


