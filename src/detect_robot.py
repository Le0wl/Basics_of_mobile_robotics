import cv2
import numpy as np
import time

class RobotPosOrient:
    def __init__(self, aruco_dict):
        self.pos = np.array([0, 0])
        self.angle = 0
        self.marker_size = 300
        self.marker_id = 1
        self.aruco_dict = aruco_dict
        self.parameters = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()

        self.id = None

    def detect_marker(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            for i in range(len(ids)):

                self.id = ids[0]
                
                # Placeholder camera parameters
                focal_length = 1000  # Example focal length (adjust as needed)
                center = (frame.shape[1] / 2, frame.shape[0] / 2)  # Center of the frame
                
                camera_matrix = np.array([[focal_length, 0, center[0]],
                                          [0, focal_length, center[1]],
                                          [0, 0, 1]], dtype=np.float64)

                dist_coeffs = np.zeros((4, 1))  # Placeholder distortion coefficients

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size, camera_matrix, dist_coeffs)

                # Get x and y positions (normalized between 0 and 1)
                x_pos = tvecs[0][0][0] / frame.shape[1]
                y_pos = tvecs[0][0][1] / frame.shape[0]

                self.pos = np.array([x_pos, y_pos])
                # Calculate the angle relative to the camera's horizontal axis (in radians)
                angle = np.arctan2(rvecs[0][0][1], rvecs[0][0][2])

                # Convert angle to degrees
                angle_deg = np.degrees(angle)
                self.angle = angle_deg
                # Draw axis manually (visualization)
                axis_length = self.marker_size / 2
                points = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]]).reshape(-1, 3, 1)
                axis_points, _ = cv2.projectPoints(points, rvecs[0], tvecs[0], camera_matrix, dist_coeffs)
                
                # Convert axis_points to integer values for drawing lines
                axis_points = axis_points.astype(int)

                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[1].ravel()), (0, 0, 255), 5)
                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[2].ravel()), (0, 255, 0), 5)
                frame = cv2.line(frame, tuple(axis_points[0].ravel()), tuple(axis_points[3].ravel()), (255, 0, 0), 5)
                
                # Additional processing if needed for each detected marker
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return frame

    def show_marker_detection(self):
        cap = cv2.VideoCapture(0)  # Use 0 for default camera, change the value for other cameras

        if not cap.isOpened():
            print("Cannot open camera")
            return
        start_time = time.time()
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            detected_frame = self.detect_marker(frame)
            cv2.imshow('Marker Detection', detected_frame)

            if time.time() - start_time > 0.5:
                # Print position and angle information
                print(f"Marker ID: {self.id}")
                print(f"X Position (normalized): {self.pos[0]:.2f}")
                print(f"Y Position (normalized): {self.pos[1]:.2f}")
                print(f"Angle relative to horizontal (degrees): {self.angle:.2f}")

                start_time = time.time()
                

            if cv2.waitKey(1) == ord('q'):  # Press 'q' to exit
                break

        cap.release()
        cv2.destroyAllWindows()


def findArucoMarkers(image, markerSize=300, totalMarkers=1):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    dictionary_key = getattr(cv2.aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_key)

    aruco_params = cv2.aruco.DetectorParameters()

    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dictionary,
                                                            parameters=aruco_params)

    return marker_corners, marker_ids


def main_detect():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    robot = RobotPosOrient(aruco_dict)
    robot.show_marker_detection()

#if __name__ == "__main__":
    #main()
