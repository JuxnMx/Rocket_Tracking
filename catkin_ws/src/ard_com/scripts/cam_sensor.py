#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import imutils

bridge = CvBridge()
cap=cv2.VideoCapture(0)
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParams = cv2.aruco.DetectorParameters_create()


def send_cam_msg():
    pub_img = rospy.Publisher('/cam_img', Image, queue_size=1)
    pub_msg = rospy.Publisher('/cam_msg', Pose, queue_size=16)
    rospy.init_node('cam_pub', anonymous=True)
    rate = rospy.Rate(4) # 4Hz=0.25 seconds
    
    # object_detector=cv2.createBackgroundSubtractorMOG2(varThreshold=50)     
    rospy.loginfo('Start Sending Arduino Message')        

    while not rospy.is_shutdown():
        ret , frame =cap.read()
        if not ret:
            break
        frame=cv2.flip(frame,1)
        width=450
        height=600
        frame = imutils.resize(frame, width=width,height=height)

        # detect ArUco original markers in the frame
        corners,ids,rejected = cv2.aruco.detectMarkers(frame,arucoDict,parameters=arucoParams)
	
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                if markerID == 58: #Rocket ID 58
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box of the ArUCo detection
                    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the
                    x = int((topLeft[0] + bottomRight[0]) / 2.0)
                    y = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)
                    cam_msg = Pose()
                    cam_msg.position.x = x
                    cam_msg.position.y = height - y
                    pub_msg.publish(cam_msg)
                # draw the ArUco marker ID on the frame
                    cv2.putText(frame, str('rocket'),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)       
        msg_img=bridge.cv2_to_imgmsg(frame,'bgr8')
        pub_img.publish(msg_img)
        
        if rospy.is_shutdown():
            cap.release()
            cv2.destroyAllWindows()
            rospy.loginfo('Stop Sending Camera Publication') 
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
	        break

if __name__ == '__main__':
    try:
        send_cam_msg()

    except rospy.ROSInterruptException:
        pass        