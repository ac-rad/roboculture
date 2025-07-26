import cv2
import numpy as np
import glob

# Camera calibration
def calibrate_camera(images, nx, ny):
    """
    Calibrate camera using chessboard images
    :param images: list of chessboard images
    :param nx: number of corners in x direction 
    :param ny: number of corners in y direction
    """
    # Prepare object points
    objp = np.zeros((nx*ny, 3), np.float32)
    objp[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1,2)

    # Arrays to store object points and image points
    objpoints = [] # 3D points in real world space
    imgpoints = [] # 2D points in image plane

    for fname in images:
        # Read in each image
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

        # If corners are found, add object points and image points
        if ret == True:

            # Refine corners
            corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    # Calibrate camera using object points and image points
    ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    # cameraMatrix: intrinsic camera matrix]
    # distCoeffs: distortion coefficients
    # rvecs: rotation vectors, 3×1 vector. The direction of the vector specifies the axis of rotation and the magnitude of the vector specifies the angle of rotation.
    # tvecs: translation vectors, 3×1 vector

    return cameraMatrix, distCoeffs


if __name__ == '__main__':

    images = glob.glob('camera_cal_imgs/*.jpg')

    # Calibrate camera
    cameraMatrix, distCoeffs = calibrate_camera(images, 9, 6)

    np.savez('camera_cal_imgs/calibration.npz', mtx=cameraMatrix, dist=distCoeffs)

    # Load camera calibration parameters
    with np.load('camera_cal_imgs/calibration.npz') as X:
        # print calib params
        print(X['mtx'])
        print(X['dist'])

        # extract fx, fy, cx, cy
        fx = X['mtx'][0,0]
        fy = X['mtx'][1,1]
        cx = X['mtx'][0,2]
        cy = X['mtx'][1,2]

        # extract k1, k2, p1, p2, k3
        k1 = X['dist'][0,0]
        k2 = X['dist'][0,1]
        p1 = X['dist'][0,2]
        p2 = X['dist'][0,3]
        k3 = X['dist'][0,4]

        u0 = cx
        v0 = cy
        px = fx
        py = fy

        print('fx: ', fx)
        print('fy: ', fy)
        print('cx: ', cx)
        print('cy: ', cy)
        print('k1: ', k1)
        print('k2: ', k2)
        print('p1: ', p1)
        print('p2: ', p2)
        print('k3: ', k3)

# [[606.98800735   0.         331.55629021]
#  [  0.         608.67807083 236.34166349]
#  [  0.           0.           1.        ]]
# [[-3.85668627e-04  1.42084216e+00 -9.93226137e-03  4.84760509e-03
#   -6.02852685e+00]]
# fx:  606.9880073466848
# fy:  608.6780708260555
# cx:  331.55629021374466
# cy:  236.34166348802154
# k1:  -0.0003856686270765646
# k2:  1.4208421639444635
# p1:  -0.009932261368307414
# p2:  0.0048476050939629655
# k3:  -6.028526848910048