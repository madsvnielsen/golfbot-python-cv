import numpy as np
import cv2 as cv
import glob

chessboardsize = (8, 6)
framesize = (1616, 1077)

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((chessboardsize[0] * chessboardsize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboardsize[0], 0:chessboardsize[1]].T.reshape(-1, 2)

objPoints = []
imgPoints = []

images = glob.glob('*.jpg')

if not images:
    print("No images found in the directory.")
else:
    for image in images:
        print(f"Processing image: {image}")
        img = cv.imread(image)
        if img is None:
            print(f"Failed to read image {image}")
            continue

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Apply Gaussian Blur to reduce noise
        gray = cv.GaussianBlur(gray, (5, 5), 0)

        ret, corners = cv.findChessboardCorners(gray, chessboardsize, None)

        if ret == True:
            objPoints.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgPoints.append(corners2)  # Append refined corners instead of the original corners

            cv.drawChessboardCorners(img, chessboardsize, corners2, ret)
            cv.imshow('img', img)
            cv.waitKey(1000)
        else:
            print(f"Chessboard corners not found in image {image}")

    cv.destroyAllWindows()

if len(objPoints) > 0 and len(imgPoints) > 0:
    ret, cameramatrix, dist, rvecs, tvecs = cv.calibrateCamera(objPoints, imgPoints, framesize, None, None)
    print("Camera Calibrated", ret)
    print("\nCamera matrix", cameramatrix)
    print("\nDistortion Parameters", dist)
    print("\nRotation Vectors", rvecs)
    print("\nTranslation vectors", tvecs)

    img1 = cv.imread('cali5.jpg')
    img2 = cv.imread('cali6.jpg')  # Assuming you have another image for homography
    gray1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
    gray2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)

    # Detect chessboard corners in the second image
    ret1, corners1 = cv.findChessboardCorners(gray1, chessboardsize, None)
    ret2, corners2 = cv.findChessboardCorners(gray2, chessboardsize, None)

    if ret1 and ret2:
        corners1 = cv.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
        corners2 = cv.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)

        # Find the homography matrix
        H, mask = cv.findHomography(corners1, corners2, cv.RANSAC)

        print("Homography matrix:")
        print(H)

        # Use the homography matrix to warp img1 to img2's plane
        height, width = img2.shape[:2]
        warped_img = cv.warpPerspective(img1, H, (width, height))
        cv.imshow('Warped Image', warped_img)
        cv.waitKey(0)
        cv.destroyAllWindows()
        cv.imwrite('warped_image.png', warped_img)
    else:
        print("Chessboard corners not found in both images, unable to compute homography.")

    img = cv.imread('cali5.jpg')
    h, w = img.shape[:2]
    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameramatrix, dist, (w, h), 1, (w, h))

    dst = cv.undistort(img, cameramatrix, dist, None, newCameraMatrix)
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    cv.imwrite('caliresults1.png', dst)

    mapx, mapy = cv.initUndistortRectifyMap(cameramatrix, dist, None, newCameraMatrix, (w, h), 5)
    dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
    dst = dst[y:y + h, x:x + w]
    cv.imwrite('caliresults2.png', dst)

    mean_error = 0
    for i in range(len(objPoints)):
        imgPoints2, _ = cv.projectPoints(objPoints[i], rvecs[i], tvecs[i], cameramatrix, dist)
        error = cv.norm(imgPoints[i], imgPoints2, cv.NORM_L2) / len(imgPoints2)
        mean_error += error

    print("\nTotal error: {}".format(mean_error / len(objPoints)))
else:
    print("Not enough points for camera calibration.")
