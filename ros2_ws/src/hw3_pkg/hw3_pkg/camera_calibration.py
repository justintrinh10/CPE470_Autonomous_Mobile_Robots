# Question 3

import numpy as np
import cv2 as cv
import glob

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points like (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0)
objp = np.zeros((6 * 8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load all jpg images in the folder
images = glob.glob('*.jpg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    print(f"{gray}")

    # Find the chessboard corners
    ret, corners = cv.findChessboardCorners(gray, (8, 6), None)

    # If found, add object points and image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (8, 6), corners2, ret)
        output_name = f"corners_{fname}"
        cv.imwrite(output_name, img)
        print(f"Saved {output_name}")

cv.destroyAllWindows()

# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print(f"Camera matrix:\n{mtx}")

# Save camera matrix for later use
np.savez('camera_matrix.npz', mtx=mtx)
print(f"Camera matrix:\n{mtx}")

# Save camera matrix to .txt file
np.savetxt('camera_matrix.txt', mtx, fmt='%.6f')
print("Camera matrix saved to camera_matrix.txt")
