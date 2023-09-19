import cv2 as cv
import glob
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
from scipy.spatial import distance



def calibrate_camera(images_folder):

    # long method using images
    images_names = glob.glob(images_folder)
    print(len(images_names))
    images = []
    for imname in images_names:
        im = cv.imread(imname, 1)
        images.append(im)

    # criteria used by checkerboard pattern detector.
    # Change this if the code can't find the checkerboard
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    CHECKERBOARD = (6,9)
    rows = CHECKERBOARD[0]  # number of checkerboard rows.
    columns = CHECKERBOARD[1]  # number of checkerboard columns.
    world_scaling = 3.3  # change this to the real world square size. Or not. square is 3.302mm

    # coordinates of squares in the checkerboard world space
    objp = np.zeros((1, 6 * 9, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
    objp = world_scaling * objp


    # frame dimensions. Frames should be the same size.
    width = images[0].shape[1]
    height = images[0].shape[0]

    # Pixel coordinates of checkerboards
    imgpoints = []  # 2d points in image plane.

    # coordinates of the checkerboard in checkerboard world space.
    objpoints = []  # 3d point in real world space
    idx = 1
    print("num of imgs: ", len(images))
    for frame in images:
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # find the checkerboard
        ret, corners = cv.findChessboardCorners(gray, (columns, rows), None)

        if ret == True:
            print('found corners in img%d'%idx)
            # Convolution size used to improve corner detection. Don't make this too large.
            conv_size = (11, 11)

            # opencv can attempt to improve the checkerboard coordinates
            corners = cv.cornerSubPix(gray, corners, conv_size, (-1, -1), criteria)
            #cv.drawChessboardCorners(frame, (columns, rows), corners, ret)
            #resized = cv.resize(frame, (1500,1000))
            #cv.imshow('img', resized)
            #cv.waitKey(10000)

            objpoints.append(objp)
            imgpoints.append(corners)
        else:
            print('could not find corners in img%d' % idx)
        idx += 1
    
    for i in range(len(imgpoints)):
        np.save('imgpoints/calib%spts/%d.npy'%(images_folder[0], i), imgpoints[i], allow_pickle=True)

    '''
    # fast version with loaded-in matrices
    # coordinates of squares in the checkerboard world space
    objp = np.zeros((6 * 9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
    objp = 1 * objp

    # coordinates of the checkerboard in checkerboard world space.
    objpoints = []  # 3d point in real world space
    for i in range(25):
        objpoints.append(objp)

    imgpoints = []
    for i in range(25):
        a = np.load('imgpoints/calibBpts/%d.npy' % i, allow_pickle=True)
        imgpoints.append(a)
    print('read %d groups of imgpoints' % (len(imgpoints)))
    '''

    #ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (2592, 1944), None, None)
    ret, mtx, dist, rvecs, tvecs = cv.fisheye.calibrate(objpoints, imgpoints, (2592, 1944), None, None, flags= cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv.fisheye.CALIB_FIX_SKEW + cv.fisheye.CALIB_FIX_K2 + cv.fisheye.CALIB_FIX_K3 + cv.fisheye.CALIB_FIX_K4)
    print('rmse:', ret)     # RMSE value below 1 is VERY good. up to 2-3 is acceptable.
    print('camera matrix:\n', mtx)
    print('distortion coeffs:', dist)
    print('Rs:\n', rvecs)
    print('Ts:\n', tvecs)

    return mtx, dist


def stereo_calibrate(mtx1, dist1, mtx2, dist2, frames_folder):

     # version with raw images
    # read the synched frames
    images_names = glob.glob(frames_folder)
    images_names = sorted(images_names)
    c1_images_names = images_names[:len(images_names) // 2]
    c2_images_names = images_names[len(images_names) // 2:]

    c1_images = []
    c2_images = []
    for im1, im2 in zip(c1_images_names, c2_images_names):
        _im = cv.imread(im1, 1)
        c1_images.append(_im)

        _im = cv.imread(im2, 1)
        c2_images.append(_im)

    # change this if stereo calibration not good.
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

    CHECKERBOARD = (6, 9)
    rows = CHECKERBOARD[0]  # number of checkerboard rows.
    columns = CHECKERBOARD[1]  # number of checkerboard columns.
    world_scaling = 3.3  # change this to the real world square size. Or not.

    # coordinates of squares in the checkerboard world space
    objp = np.zeros((rows * columns, 3), np.float32)
    objp[:, :2] = np.mgrid[0:rows, 0:columns].T.reshape(-1, 2)
    objp = world_scaling * objp

    # frame dimensions. Frames should be the same size.
    width = c1_images[0].shape[1]
    height = c1_images[0].shape[0]

    # Pixel coordinates of checkerboards
    imgpoints_left = []  # 2d points in image plane.
    imgpoints_right = []

    # coordinates of the checkerboard in checkerboard world space.
    objpoints = []  # 3d point in real world space

    for frame1, frame2 in zip(c1_images, c2_images):
        gray1 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
        gray2 = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
        c_ret1, corners1 = cv.findChessboardCorners(gray1, (6, 9), None)
        c_ret2, corners2 = cv.findChessboardCorners(gray2, (6, 9), None)

        if c_ret1 == True and c_ret2 == True:
            corners1 = cv.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
            corners2 = cv.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)

            # cv.drawChessboardCorners(frame1, (6, 9), corners1, c_ret1)
            # cv.imshow('img', frame1)

            # cv.drawChessboardCorners(frame2, (6, 9), corners2, c_ret2)
            # cv.imshow('img2', frame2)
            # cv.waitKey(500)

            objpoints.append(objp)
            imgpoints_left.append(corners1)
            imgpoints_right.append(corners2)

    for i in range(len(imgpoints_left)):
        np.save('imgpoints/stereocalibApts/%d.npy' % i, imgpoints_left[i], allow_pickle=True)
    for i in range(len(imgpoints_right)):
        np.save('imgpoints/stereocalibBpts/%d.npy' % i, imgpoints_right[i], allow_pickle=True)
    #end of raw image version
    '''

    # loading in values from disk
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

    CHECKERBOARD = (6, 9)
    rows = CHECKERBOARD[0]  # number of checkerboard rows.
    columns = CHECKERBOARD[1]  # number of checkerboard columns.
    world_scaling = 3.302  # change this to the real world square size. Or not. square is 3.302mm

    # coordinates of squares in the checkerboard world space
    objp = np.zeros((rows * columns, 3), np.float32)
    objp[:, :2] = np.mgrid[0:rows, 0:columns].T.reshape(-1, 2)
    objp = world_scaling * objp

    objpoints = []  # 3d point in real world space
    for i in range(15):
        objpoints.append(objp)

    imgpoints_left = []
    for i in range(15):
        a = np.load('imgpoints/stereocalibApts/%d.npy' % i, allow_pickle=True)
        imgpoints_left.append(a)
    print('STEREO read %d groups of camera A imgpoints' % (len(imgpoints_left)))

    imgpoints_right = []
    for i in range(15):
        a = np.load('imgpoints/stereocalibBpts/%d.npy' % i, allow_pickle=True)
        imgpoints_right.append(a)
    print('STEREO: read %d groups of camera B imgpoints' % (len(imgpoints_right)))
    # end of reading version
    '''

    stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
    ret, CM1, dist1, CM2, dist2, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx1,
                                                                 dist1,
                                                                 mtx2, dist2, (2592, 1944), criteria=criteria,
                                                                 flags=stereocalibration_flags)

    print(ret)
    np.save('R.npy', R, allow_pickle=True)
    np.save('T.npy', T, allow_pickle=True)
    return R, T


def triangulate(mtx1, mtx2, R, T, ptA, ptB):
    pointsA = [[162,136]]               # list of lists where first item is the x,y coordinates of the target from camA
    pointsA.append(ptA)                 # adding x,y coordinates of tip of catheter from camA to the list.
    pointsB = [[120,175]]               # list of lists where first item is the x,y coordinates of the target from camB
    pointsB.append(ptB)                 # adding x,y coordinates of tip of catheter from camB to the list.
    pointsA = np.array(pointsA)
    pointsB = np.array(pointsB)

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
    for pointA, pointB in zip(pointsA, pointsB):
        _p3d = DLT(P1, P2, pointA, pointB)
        p3ds.append(_p3d)
    p3ds = np.array(p3ds)
    
    dst = distance.euclidean(p3ds[0],p3ds[1])
    #print('Distance to target point:',dst)
    
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d(-70, -50)
    ax.set_ylim3d(-80, -50)
    ax.set_zlim3d(0, 50)

    #connections = [[0,1]]
    ax.plot(p3ds[0][0],p3ds[0][1], 'ro', color='r')
    ax.plot(p3ds[1][0],p3ds[1][1], 'ro', color='g')
    ax.set_title('This figure can be rotated.')
    # uncomment to see the triangulated pose. This may cause a crash if you're also using cv.imshow() above.
    plt.show()
    '''
    return dst


mtx1, dist1 = calibrate_camera(images_folder='A/*')
mtx2, dist2 = calibrate_camera(images_folder='B/*')

np.save('mtxA.npy', mtx1, allow_pickle=True)
np.save('distA.npy', dist1, allow_pickle=True)
np.save('mtxB.npy', mtx2, allow_pickle=True)
np.save('distB.npy', dist2, allow_pickle=True)


mtx1 = np.load('mtxA.npy', allow_pickle=True)
dist1 = np.load('distA.npy', allow_pickle=True)
mtx2 = np.load('mtxB.npy', allow_pickle=True)
dist2 = np.load('distB.npy', allow_pickle=True)

R, T = stereo_calibrate(mtx1, dist1, mtx2, dist2, 'synched/*')
np.save('R.npy', R, allow_pickle=True)
np.save('T.npy', T, allow_pickle=True)

# this call might cause segmentation fault error. This is due to calling cv.imshow() and plt.show()
#print(triangulate(mtx1, mtx2, R, T))
