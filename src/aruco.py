import cv2
import numpy as np
import time
from constants import *

Map_camera = np.zeros((UNIT_NUMBER,UNIT_NUMBER,2))

class ArucoMarker:
    """
    This class represents an ArUco marker. 
    There are 5 in total so we create 5 objects of this class each with their unique ID.
    We Have the position of the center of the marker and the angle of the marker in the camera frame
    """

    def __init__(self, marker_id):
        self.pos = np.array([0, 0])
        self.angle = 0
        self.marker_size = 300
        self.marker_id = marker_id
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
        self.marker_pxl_size = 0
        self.focal_length = 1000  # Example focal length (adjust as needed)
        self.center = (640/2, 480/2)  # Example center of the frame, replace with actual center
        self.goal_idx = np.array([0, 0])



        self.num_frames_average_red = 30   # Adjust this value
        self.max_tracked_objects_red = 2   # Adjust this value
        self.red_avg_count = 0
        self.avg_cx_red = [0] * self.max_tracked_objects_red
        self.avg_cy_red = [0] * self.max_tracked_objects_red
        self.num_tracked_objects_red = 0
        self.pos_red = np.array([[0,0],[0,0]])

    def update_marker(self, frame):
        #Placeholder camera parameters
        focal_length = 1000  # Example focal length (adjust as needed)
        center = (frame.shape[1] / 2, frame.shape[0] / 2)  # Center of the frame
        # Placeholder camera matrix 
        cameraMatrix = np.array([[focal_length, 0, center[0]],
                                     [0, focal_length, center[1]],
                                    [0, 0, 1]], dtype=np.float64)

        distCoeffs = np.zeros((4, 1))  
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None and self.marker_id in ids:
            idx = np.where(ids == self.marker_id)[0][0]
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[idx], self.marker_size, cameraMatrix, distCoeffs)


            self.marker_pxl_size = np.linalg.norm(corners[idx][0][0] - corners[idx][0][1])

            axis_length = self.marker_size / 2
            points = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]]).reshape(-1, 3, 1)
            axis_points, _ = cv2.projectPoints(points, rvecs[0], tvecs[0], cameraMatrix, distCoeffs)
            axis_points = axis_points.astype(int)

            angle = np.arctan2(axis_points[0].ravel()[0] - axis_points[1].ravel()[0], axis_points[0].ravel()[1] - axis_points[1].ravel()[1])
            angle = np.degrees(angle) + 90
            self.pos = axis_points[0].ravel()
            # angle goes from 0 to 270 ang 0 to -90
            if(angle > 180):
                angle = angle - 360
            self.angle = angle



            if(self.marker_id == 1):
                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[1].ravel()), (0, 255, 0), 2)
            #else:
                #frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[1].ravel()), (0, 0, 100), 5)
                #frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[2].ravel()), (0, 100, 0), 5)
                #frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[3].ravel()), (100, 0, 0), 5)

            if(self.marker_id == 4):
                #cv2.putText(frame,"o",(axis_points[1][0][0] ,axis_points[1][0][1]),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2,cv2.LINE_AA)
                # display circle in the center of the marker (self.pos)
               
                #display all the unit positions
                for i in range(UNIT_NUMBER):
                    for j in range(UNIT_NUMBER):
                        unit_pos = np.array([Map_camera[i][j][0], Map_camera[i][j][1]]) 

                        # Ensure unit_pos contains integer values and convert to tuple
                        unit_pos = tuple(map(int, unit_pos))

                        # Check if unit_pos is within frame boundaries
                        if 0 <= unit_pos[0] < frame.shape[1] and 0 <= unit_pos[1] < frame.shape[0]:
                            if i == self.goal_idx[0] and j == self.goal_idx[1]:
                                frame = cv2.circle(frame, unit_pos, 3, (0, 0, 255), -1)
                            else:
                                frame = cv2.circle(frame, unit_pos, 1, (255, 0, 0), -1)
                            #frame = cv2.circle(frame, (340,100), 1, (0, 255, 0), -1)
                            #print("UNIT: ",unit_pos)
        

        return frame
    
    
    def detect_red_objects(self,frame):
        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define the lower and upper bounds for the red color in HSV
        lower_red = np.array([0, 20, 10])
        upper_red = np.array([10, 255, 255])

        # Create a mask to isolate red regions in the image
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours of red objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Loop through the contours to find the bounding boxes of red objects
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Set a minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                if w > 30 and h > 30:  # Set minimum width and height thresholds
                    top_left = (x, y)
                    bottom_right = (x + w, y + h)
                    # Draw contours around the detected objects on the frame
                    cv2.rectangle(frame, top_left, bottom_right, (0, 0, 200), 2)

        return frame

def main_aruco(*markers):
    cap = cv2.VideoCapture(0)  # Use 0 for default camera, change the value for other cameras

    if not cap.isOpened():
        print("Cannot open camera")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
    
        for marker in markers:
            frame = marker.update_marker(frame)
            #print(f"Marker ID: {marker.marker_id}, Angle: {marker.angle:.2f}")
        frame = marker.detect_red_objects(frame)  # Create a copy to preserve the original frame

        cv2.imshow('Markers Detection', frame)

        if cv2.waitKey(1) == ord('q'):  # Press 'q' to exit
            break

    cap.release()
    cv2.destroyAllWindows()

def set_map(map):
    global Map_camera
    Map_camera = map