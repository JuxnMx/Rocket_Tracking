# Copyright @JuxnMx 2022
import imutils
import cv2

def init_pose():
    x=0.0
    y=0.0
    cap=cv2.VideoCapture(0)
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv2.aruco.DetectorParameters_create()
    while (x==0.0) and (y==0.0):
        _,frame = cap.read()
        width=450
        height=600
        frame = imutils.resize(frame, width=width,height=height)
        frame=cv2.flip(frame,1)
        
        # detect ArUco original markers in the frame
        corners,ids,rejected = cv2.aruco.detectMarkers(frame,arucoDict,parameters=arucoParams)
        
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                topLeft, _, bottomRight, _ = corners
                if markerID == 58: #Rocket Marker ID: 58
                    x = int((topLeft[0] + bottomRight[0]) / 2.0)
                    y = height - int((topLeft[1] + bottomRight[1]) / 2.0)
    return x,y