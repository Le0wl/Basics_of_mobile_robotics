import cv2
import numpy as np
import time

Map_camera = np.zeros((6,6,2))
UNIT_NUMBER = 4

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


        self.num_frames_average_black = 30   # Adjust this value
        self.max_tracked_objects_black = 8
        self.blk_avg_count = 0
        self.avg_cx_black = [0] * self.max_tracked_objects_black
        self.avg_cy_black = [0] * self.max_tracked_objects_black
        self.num_tracked_objects = 0

    def update_marker(self, frame):
        #Placeholder camera parameters
        focal_length = 1000  # Example focal length (adjust as needed)
        center = (frame.shape[1] / 2, frame.shape[0] / 2)  # Center of the frame
        # Placeholder camera matrix 
        cameraMatrix = np.array([[focal_length, 0, center[0]],
                                     [0, focal_length, center[1]],
                                    [0, 0, 1]], dtype=np.float64)

        distCoeffs = np.zeros((4, 1))  # Placeholder distortion coefficients
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        #self.marker_size = np.linalg.norm(corners[0][0][0] - corners[0][0][1])
        
        if ids is not None and self.marker_id in ids:
            idx = np.where(ids == self.marker_id)[0][0]
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[idx], self.marker_size, cameraMatrix, distCoeffs)


            self.marker_pxl_size = np.linalg.norm(corners[idx][0][0] - corners[idx][0][1])
            #x_pos = tvecs[0][0][0] #/ frame.shape[1] + 0.5 
            #y_pos = tvecs[0][0][1] #/ frame.shape[0] + 0.5
            # center of the image is 320 and 240. We set the position in this referential
            #x_pos = tvecs[0][0][0] + 320
            #y_pos = tvecs[0][0][1] + 240

            #self.pos = np.array([x_pos, y_pos])

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
                #cv2.putText(frame,"Thymio",(axis_points[1][0][0],axis_points[1][0][1]),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)
                #print("Marker ID: ", self.marker_id, "Angle: ", self.angle)
            else:
                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[1].ravel()), (0, 0, 100), 5)
                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[2].ravel()), (0, 100, 0), 5)
                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[3].ravel()), (100, 0, 0), 5)

            if(self.marker_id == 4):
                cv2.putText(frame,"o",(axis_points[1][0][0] ,axis_points[1][0][1]),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2,cv2.LINE_AA)
                # display circle in the center of the marker (self.pos)
                #frame = cv2.circle(frame, tuple(self.pos), 5, (0, 0, 255), -1)
                """
                for i in range(3):

                    unit_pos = np.array([Map_camera[i][0][0], Map_camera[i][0][1]]) 
                    # + np.array([self.marker_pxl_size/2, 0]) + np.array([Map_camera[i][0][0], 0]) 

                    # Ensure unit_pos contains integer values and convert to tuple
                    unit_pos = tuple(map(int, unit_pos))
                    
                    #unit_pos = self.pos

                    # Check if unit_pos is within frame boundaries
                    if 0 <= unit_pos[0] < frame.shape[1] and 0 <= unit_pos[1] < frame.shape[0]:
                        frame = cv2.circle(frame, unit_pos, 5, (0, 0, 255), -1)
                    """
                #display all the unit positions
                for i in range(UNIT_NUMBER):
                    for j in range(UNIT_NUMBER):
                        unit_pos = np.array([Map_camera[i][j][0], Map_camera[i][j][1]]) 
                        # + np.array([self.marker_pxl_size/2, 0]) + np.array([Map_camera[i][0][0], 0]) 

                        # Ensure unit_pos contains integer values and convert to tuple
                        unit_pos = tuple(map(int, unit_pos))
                        
                        #unit_pos = self.pos

                        # Check if unit_pos is within frame boundaries
                        if 0 <= unit_pos[0] < frame.shape[1] and 0 <= unit_pos[1] < frame.shape[0]:
                            if i == 0 and j == 3:
                                frame = cv2.circle(frame, unit_pos, 5, (0, 0, 255), -1)
                            else:
                                frame = cv2.circle(frame, unit_pos, 2, (255, 0, 0), -1)
                            #frame = cv2.circle(frame, (340,100), 1, (0, 255, 0), -1)
                            #print("UNIT: ",unit_pos)
        

        return frame
    
    
    def track_black(self, frame):

        ## ----------Detecting Black Objects----------
        
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
        # Apply adaptive thresholding
        # . Pixels with intensity greater than 45 are set to 255 (white), and others are set to 0 (black). 
        # Lowering the threshold to detect darker colors
        _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

        # Apply morphological operations to reduce noise
        # Small kernel = track small objects, large kernel = more likely to smooth out details and may help in reducing noise or small variations
        kernel = np.ones((5, 5), np.uint8)  # creates a 5x5 square shaped kernel
        # Changing the iterations will help to improve the noise. 
        # Each iteration of the operation modifies the image, and the result becomes the input for the next iteration.
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)

        # ------------Detecting and averaging centroid----------
        # Find contours in the binary image
        contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours based on area in descending order and count the number of tracked objects
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)[:self.max_tracked_objects_black]

        # Update the number of tracked objects
        num_tracked_objects = min(len(sorted_contours), self.max_tracked_objects_black)

        # Calculate the average centroid value
        for i, contour in enumerate(sorted_contours):

            M_black = cv2.moments(contour)
            if M_black["m00"] != 0:
                cx_black = int(M_black["m10"] / M_black["m00"])
                cy_black = int(M_black["m01"] / M_black["m00"])
            else:
                cx_black, cy_black = 0, 0

            if i < self.max_tracked_objects_black:
                self.avg_cx_black[i] += cx_black
                self.avg_cy_black[i] += cy_black

            # Draw a dot at the centroid
            cv2.circle(frame, (cx_black, cy_black), 2, (120, 0, 120), -1)

            # Draw the bounding box
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (120, 0, 120), 2)

            # Add the label "Obstacle" inside the bounding box
            label = "Obstacle".format(cx_black, cy_black)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)

        # Counter for average centroid
        self.blk_avg_count += 1
        # print("Black counter: {}".format(blk_avg_count))

        if self.blk_avg_count == self.num_frames_average_black:
            for i in range(self.num_tracked_objects):
                self.avg_cx_black[i] = round(self.avg_cx_black[i] / self.num_frames_average_black, 2)
                self.avg_cy_black[i] = round(self.avg_cy_black[i] / self.num_frames_average_black, 2)
                #print("Obstacle_Center {}: ({}, {})".format(i+1, self.avg_cx_black[i], self.avg_cy_black[i]))


            # Reset the accumulated values and counter for the next set of frames
            self.blk_avg_count = 0
            self.avg_cx_black = [0] * self.max_tracked_objects_black
            self.avg_cy_black = [0] * self.max_tracked_objects_black
                


        return frame

def main_aruco(*markers):
    cap = cv2.VideoCapture(0)  # Use 0 for default camera, change the value for other cameras

    if not cap.isOpened():
        print("Cannot open camera")
        return

    #markers = [ArucoMarker(marker_id) for marker_id in range(1, 6)]  # Create instances for each ArUco marker

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
    
        for marker in markers:
            frame = marker.update_marker(frame)
            #print(f"Marker ID: {marker.marker_id}, Angle: {marker.angle:.2f}")
        frame = marker.track_black(frame)  # Create a copy to preserve the original frame

        cv2.imshow('Markers Detection', frame)

        if cv2.waitKey(1) == ord('q'):  # Press 'q' to exit
            break

    cap.release()
    cv2.destroyAllWindows()

def set_map(map):
    global Map_camera
    Map_camera = map