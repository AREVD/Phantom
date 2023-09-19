import RPi.GPIO as gp
import numpy as np
from scipy import linalg
from scipy.spatial import distance
import os
import time
import cv2

gp.setwarnings(False)
gp.setmode(gp.BOARD)

gp.setup(7, gp.OUT)
gp.setup(11, gp.OUT)
gp.setup(12, gp.OUT)

ptA = []
ptB = []

def triangulate(mtx1, mtx2, R, T, ptA, ptB):                            # calculating distance between hole and target.
    pointA = [[159,152]]                # camA location of target
    pointA.append(ptA)
    pointB = [[111,206]]                # camB location of target
    pointB.append(ptB)
    pointA = np.array(pointA)           # camA location of tip
    pointB = np.array(pointB)           # camB location of tip

    RT1 = np.concatenate([np.eye(3), [[0], [0], [0]]], axis=-1)
    P1 = mtx1 @ RT1  # projection matrix for C1

    # RT matrix for C2 is the R and T obtained from stereo calibration.
    RT2 = np.concatenate([R, T], axis=-1)
    P2 = mtx2 @ RT2  # projection matrix for C2

    def DLT(P1, P2, point1, point2):        # DLT triangulation alg.

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

    p3ds = []                               # set of 3d points
    for pointA, pointB in zip(pointA, pointB):
        _p3d = DLT(P1, P2, pointA, pointB)  # computing 3d points
        p3ds.append(_p3d)
    p3ds = np.array(p3ds)
    
    dst = distance.euclidean(p3ds[0],p3ds[1])       # distance between the 2 points
    #print('Distance to target point:',dst)
    
    return (p3ds[1], dst)   # tuple of tip coords and distance to target

# function to display the coordinates of
# of the points clicked on the image
def click_eventA(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        global ptA
        # scaling coordinates down to triangulation scale
        ptA = [x/2.5, y/2.5]
        cv2.imshow('image', cv2.circle(img, (x,y), radius=3, color=(255,0,0), thickness=-1) )

        
def click_eventB(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        global ptB
        # scaling coordinates down to triangulation scale
        ptB = [x/2.5, y/2.5]
        cv2.imshow('image', cv2.circle(img, (x,y), radius=3, color=(255,0,0), thickness=-1) )


def capture(cam):
    #cmd = "libcamera-still -t 0"
    cmd = "libcamera-still -t 1 --width 800 --height 600 -o capture_%s.jpg" % cam
    os.system(cmd)

if __name__ == "__main__":
    gp.setmode(gp.BOARD)
    gp.setup(32,gp.OUT)
    gp.output(32,gp.HIGH)
    
    print('Taking photo with camera A')
    i2c = "i2cset -y 1 0x70 0x00 0x04"
    os.system(i2c)
    gp.output(7, False)
    gp.output(11, False)
    gp.output(12, True)
    capture('A')
    print('Taking photo with camera B')
    i2c = "i2cset -y 1 0x70 0x00 0x06"
    os.system(i2c)
    #gp.output(7, False)
    gp.output(11, True)
    gp.output(12, False)
    capture('B')
    
    gp.output(7, False)
    gp.output(11, False)
    gp.output(12, True)
    

    # reading the image
    img = cv2.imread('capture_A.jpg', 1)
    
    # displaying the image
    cv2.imshow('image', img)

    # setting mouse handler for the image
    # and calling the click_event() function
    cv2.setMouseCallback('image', click_eventA)

    # wait for a key to be pressed to exit
    cv2.waitKey(0)

    # close the window
    cv2.destroyAllWindows()
    
    # reading the image
    img = cv2.imread('capture_B.jpg', 1)
    
    # displaying the image
    cv2.imshow('image', img)

    # setting mouse handler for the image
    # and calling the click_event() function
    cv2.setMouseCallback('image', click_eventB)

    # wait for a key to be pressed to exit
    cv2.waitKey(0)

    # close the window
    cv2.destroyAllWindows()
    
    print('ptA:', ptA, 'ptB:', ptB)
    
    #triangulating
    mtxA = np.load('mtxA.npy', allow_pickle=True)    # loading cam calib
    mtxB = np.load('mtxB.npy', allow_pickle=True)
    R = np.load('R.npy', allow_pickle=True)
    T = np.load('T.npy', allow_pickle=True)
    tri = triangulate(mtxA, mtxB, R, T, ptA, ptB)
    dst = (abs(6.6*round(tri[1],2)-1.7))
    dst_str = str('%.2f'%dst)
    print('suggested distance to target: %s mm'%dst_str)
    
