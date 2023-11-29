import cv2
import numpy as np
import time

class ArucoMarker:
    def __init__(self, marker_id):
        self.pos = np.array([0, 0])
        self.angle = 0
        self.marker_size = 300
        self.marker_id = marker_id
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
        self.focal_length = 1000  # Example focal length (adjust as needed)
        self.center = (640, 480)  # Example center of the frame, replace with actual center

    def update_marker(self, frame):

        #Placeholder camera parameters
        focal_length = 1000  # Example focal length (adjust as needed)
        center = (frame.shape[1] / 2, frame.shape[0] / 2)  # Center of the frame
                
        cameraMatrix = np.array([[focal_length, 0, center[0]],
                                     [0, focal_length, center[1]],
                                    [0, 0, 1]], dtype=np.float64)

        distCoeffs = np.zeros((4, 1))  # Placeholder distortion coefficients
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None and self.marker_id in ids:
            idx = np.where(ids == self.marker_id)[0][0]
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[idx], self.marker_size, cameraMatrix, distCoeffs)
            
            # Calculate position and angle
            #x_pos = tvecs[0][0][0] / frame.shape[1]
            #y_pos = tvecs[0][0][1] / frame.shape[0]
            #give position of marker from 0 to 1 in camera frame with origin in bottom left
            x_pos = tvecs[0][0][0] / frame.shape[1] + 0.5 
            y_pos = tvecs[0][0][1] / frame.shape[0] + 0.5
            self.pos = np.array([x_pos, y_pos])

            #angle = np.arctan2(rvecs[0][0][0], rvecs[0][0][1])
            # give angle of x direction of marker in camera frame
            #angle = np.arctan2(tvecs[0][0][0], tvecs[0][0][1])
            #angle = rvecs[0][0][0]
            #self.angle = np.degrees(angle)

            # Draw axis manually (visualization)
            axis_length = self.marker_size / 2
            points = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]]).reshape(-1, 3, 1)
            axis_points, _ = cv2.projectPoints(points, rvecs[0], tvecs[0], cameraMatrix, distCoeffs)
            axis_points = axis_points.astype(int)

            angle = np.arctan2(axis_points[0].ravel()[0] - axis_points[1].ravel()[0], axis_points[0].ravel()[1] - axis_points[1].ravel()[1])
            angle = np.degrees(angle) + 90
            # angle goes from 0 to 270 ang 0 to -90
            if(angle > 180):
                angle = angle - 360
            self.angle = angle
            


            if(self.marker_id == 1):
                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[1].ravel()), (0, 0, 255), 5)
                cv2.putText(frame,"Thymio",(axis_points[1][0][0],axis_points[1][0][1]),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2,cv2.LINE_AA)
                print("Marker ID: ", self.marker_id, "Angle: ", self.angle)
            else:
                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[1].ravel()), (0, 0, 100), 5)
                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[2].ravel()), (0, 100, 0), 5)
                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[3].ravel()), (100, 0, 0), 5)
            if(self.marker_id == 4):
                cv2.putText(frame,"origin",(axis_points[1][0][0] ,axis_points[1][0][1]),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2,cv2.LINE_AA)


            

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

        cv2.imshow('Markers Detection', frame)

        if cv2.waitKey(1) == ord('q'):  # Press 'q' to exit
            break

    cap.release()
    cv2.destroyAllWindows()

