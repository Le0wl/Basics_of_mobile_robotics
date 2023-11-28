import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
from robot import robot

__init__ = ['ObjectTracker']

class ObjectTracker:
    def __init__(self):


        self.frame_height = 0
        self.frame_width = 0
        self.start_time = 0
        self.video = None

        # Initialize variables for averaging
        self.num_frames_average_black = 30   # Adjust this value
        self.max_tracked_objects_black = 8
        self.blk_avg_count = 0
        self.avg_cx_black = [0] * self.max_tracked_objects_black
        self.avg_cy_black = [0] * self.max_tracked_objects_black
        self.num_tracked_objects = 0

        self.num_frames_average_blue = 30   # Adjust this value
        self.max_tracked_objects_blue = 1   # Adjust this value
        self.blue_avg_count = 0
        self.avg_cx_blue = [0] * self.max_tracked_objects_blue
        self.avg_cy_blue = [0] * self.max_tracked_objects_blue
        self.num_tracked_objects_blue = 0

        self.num_frames_average_red = 30   # Adjust this value
        self.max_tracked_objects_red = 2   # Adjust this value
        self.red_avg_count = 0
        self.avg_cx_red = [0] * self.max_tracked_objects_red
        self.avg_cy_red = [0] * self.max_tracked_objects_red
        self.num_tracked_objects_red = 0
        self.pos_red = [0] * self.max_tracked_objects_red

        self.num_frames_average_green = 100   # Adjust this value
        self.max_tracked_objects_green = 4    # Limit green tracking to 4 objects
        self.green_avg_count = 0
        self.avg_cx_green = [0] * self.max_tracked_objects_green
        self.avg_cy_green = [0] * self.max_tracked_objects_green
        self.num_tracked_objects_green = 0

        self.detected_centroids = []

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

    def track_blue(self, frame):
        ## ----------Detecting Blue Objects----------
        
        # Convert the frame from BGR to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the blue color
        # defined as HSV (Hue, Saturation, Value)
        lower_blue = np.array([90, 100, 100])
        upper_blue = np.array([130, 255, 255])

        # Create a mask for the blue color
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Apply morphological operations to reduce noise
        kernel = np.ones((7, 7), np.uint8)
        opening_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, iterations=2)
        closing_blue = cv2.morphologyEx(opening_blue, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Find contours in the binary image
        contours_blue, _ = cv2.findContours(closing_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours based on area in descending order and count the number of tracked objects
        sorted_contours_blue = sorted(contours_blue, key=cv2.contourArea, reverse=True)[:self.max_tracked_objects_blue]

        # Update the number of tracked objects
        self.num_tracked_objects_blue = min(len(sorted_contours_blue), self.max_tracked_objects_blue)

        # Calculate the average centroid value
        for i, contour_blue in enumerate(sorted_contours_blue):

            M_blue = cv2.moments(contour_blue)
            if M_blue["m00"] != 0:
                cx_blue = int(M_blue["m10"] / M_blue["m00"])
                cy_blue = int(M_blue["m01"] / M_blue["m00"])
            else:
                cx_blue, cy_blue = 0, 0

            if i < self.max_tracked_objects_blue:
                self.avg_cx_blue[i] += cx_blue
                self.avg_cy_blue[i] += cy_blue

            # Draw a dot at the centroid
            cv2.circle(frame, (cx_blue, cy_blue), 2, (255, 0, 0), -1)

            # Draw the bounding box
            x, y, w, h = cv2.boundingRect(contour_blue)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

            # Add the label "Blue Object" inside the bounding box
            label = "Goal".format(cx_blue, cy_blue)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)

        # Counter for average centroid
        self.blue_avg_count += 1

        if self.blue_avg_count == self.num_frames_average_blue:
            for i in range(self.num_tracked_objects_blue):
                self.avg_cx_blue[i] = round(self.avg_cx_blue[i] / self.num_frames_average_blue, 2)
                self.avg_cy_blue[i] = round(self.avg_cy_blue[i] / self.num_frames_average_blue, 2)
                #print("Goal_Center {}: ({}, {})".format(i+1, self.avg_cx_blue[i], self.avg_cy_blue[i]))

            # Reset the accumulated values and counter for the next set of frames
            self.blue_avg_count = 0
            self.avg_cx_blue = [0] * self.max_tracked_objects_blue
            self.avg_cy_blue = [0] * self.max_tracked_objects_blue


        return frame

    def track_red(self, frame):
        ## ----------Detecting Red Objects----------
        
        # Convert the frame from BGR to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the red color
        # defined as HSV (Hue, Saturation, Value)
        lower_red = np.array([0, 80, 120])
        upper_red = np.array([10, 255, 255])

        # Create a mask for the red color
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        opening_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, iterations=1)
        closing_red = cv2.morphologyEx(opening_red, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Find contours in the binary image
        contours_red, _ = cv2.findContours(closing_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours based on area in descending order and count the number of tracked objects
        sorted_contours_red = sorted(contours_red, key=cv2.contourArea, reverse=True) #[:max_tracked_objects_red]

        # Update the number of tracked objects
        self.num_tracked_objects_red = min(len(sorted_contours_red), self.max_tracked_objects_red)

        centroid_red = []
        # Calculate the average centroid value
        for i, contour_red in enumerate(sorted_contours_red):

            M_red = cv2.moments(contour_red)
            if M_red["m00"] != 0:
                cx_red = int(M_red["m10"] / M_red["m00"])
                cy_red = int(M_red["m01"] / M_red["m00"])
            else:
                cx_red, cy_red = 0, 0

            if i < self.max_tracked_objects_red:
                self.avg_cx_red[i] += cx_red
                self.avg_cy_red[i] += cy_red

            #append centroid position to the list
            centroid_red.append((cx_red, cy_red))

            # Draw a dot at the centroid
            cv2.circle(frame, (cx_red, cy_red), 2, (0, 0, 255), -1)

            # Draw the bounding box
            x, y, w, h = cv2.boundingRect(contour_red)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

            # Add the label "Red Object" inside the bounding box
            label = "T".format(cx_red, cy_red)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

        # Counter for average centroid
        self.red_avg_count += 1
        

        if self.red_avg_count == self.num_frames_average_red:
            for i in range(self.num_tracked_objects_red):
                self.avg_cx_red[i] = round(self.avg_cx_red[i] / self.num_frames_average_red, 2)
                self.avg_cy_red[i] = round(self.avg_cy_red[i] / self.num_frames_average_red, 2)
                #print("Thymio {}: ({}, {})".format(i+1, self.avg_cx_red[i], self.avg_cy_red[i]))

            # Reset the accumulated values and counter for the next set of frames
            self.red_avg_count = 0
            self.avg_cx_red = [0] * self.max_tracked_objects_red
            self.avg_cy_red = [0] * self.max_tracked_objects_red
            
            self.pos_red = centroid_red

            return centroid_red    


        return frame

    def assign_coordinates(self, frame_width, frame_height):

        coordinates = []

        # Assign coordinates based on pixel size
        for i in range(self.num_tracked_objects_green):
            x_ratio = self.avg_cx_green[i] / frame_width
            y_ratio = self.avg_cy_green[i] / frame_height

            if x_ratio < 0.5 and y_ratio < 0.5:
                coordinates.append(("top_left", (self.avg_cx_green[i], self.avg_cy_green[i])))
            elif x_ratio >= 0.5 and y_ratio < 0.5:
                coordinates.append(("top_right", (self.avg_cx_green[i], self.avg_cy_green[i])))
            elif x_ratio < 0.5 and y_ratio >= 0.5:
                coordinates.append(("bottom_left", (self.avg_cx_green[i], self.avg_cy_green[i])))
            else:
                coordinates.append(("bottom_right", (self.avg_cx_green[i], self.avg_cy_green[i])))

        return coordinates
    
    def track_green(self, frame):

        # Convert the frame from BGR to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the green color
        lower_green = np.array([40, 40, 40])  # Adjust these values based on your specific shade of green
        upper_green = np.array([80, 255, 255])

        # Create a mask for the green color
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Apply morphological operations to reduce noise
        kernel = np.ones((7, 7), np.uint8)
        opening_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel, iterations=1)
        closing_green = cv2.morphologyEx(opening_green, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Find contours in the binary image
        contours_green, _ = cv2.findContours(closing_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours based on area in descending order and count the number of tracked objects
        sorted_contours_green = sorted(contours_green, key=cv2.contourArea, reverse=True)[:self.max_tracked_objects_green]

        # Update the number of tracked objects
        self.num_tracked_objects_green = min(len(sorted_contours_green), self.max_tracked_objects_green)

        # Calculate the average centroid value
        for i, contour_green in enumerate(sorted_contours_green):
            M_green = cv2.moments(contour_green)
            if M_green["m00"] != 0:
                cx_green = int(M_green["m10"] / M_green["m00"])
                cy_green = int(M_green["m01"] / M_green["m00"])
            else:
                cx_green, cy_green = 0, 0

            if i < self.max_tracked_objects_green:
                self.avg_cx_green[i] = round(self.avg_cx_green[i] * 0.9 + cx_green * 0.1, 2)  # Apply exponential moving average
                self.avg_cy_green[i] = round(self.avg_cy_green[i] * 0.9 + cy_green * 0.1, 2)

            # Draw a dot at the centroid
            cv2.circle(frame, (cx_green, cy_green), 2, (0, 255, 0), -1)

            # Draw the bounding box
            x, y, w, h = cv2.boundingRect(contour_green)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Add the label "Green Object" inside the bounding box
            label = "Green Object {}: ({}, {})".format(i+1, self.avg_cx_green[i], self.avg_cy_green[i])
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

        # Counter for average centroid
        self.green_avg_count += 1

        if self.green_avg_count == self.num_frames_average_green:
            #for i in range(self.num_tracked_objects_green):
                #print("Green Object {}: ({}, {})".format(i+1, self.avg_cx_green[i], self.avg_cy_green[i]))

            # Assign and print coordinates based on pixel size
            frame_width = frame.shape[1]
            frame_height = frame.shape[0]
            coordinates = self.assign_coordinates(frame_width, frame_height)
            #for coord in coordinates:
                # Assuming coord is a tuple of length 3
                #if len(coord) >= 3:
                    #print("Coordinate {}: {}".format(coord[0], (coord[1], coord[2])))
                #else:
                    #print("Invalid coordinate format: {}".format(coord))

            # Reset the accumulated values and counter for the next set of frames
            self.green_avg_count = 0
            self.avg_cx_green = [0] * self.max_tracked_objects_green
            self.avg_cy_green = [0] * self.max_tracked_objects_green

        return frame

    def camera_blocked(self, frame):
                 # Convert the frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Calculate the average brightness of the frame
        average_brightness = np.mean(gray_frame)

        # Set a threshold for darkness (you can adjust this threshold based on your requirements)
        darkness_threshold = 50

        # Check if the average brightness is below the darkness threshold
        return average_brightness < darkness_threshold

    def main(self):  # Get the video file and read it
        self.video = cv2.VideoCapture(1)
        ret, frame = self.video.read()

        self.frame_height, self.frame_width = frame.shape[:2]
        # Resize the video for a more convenient view
        frame = cv2.resize(frame, [self.frame_width // 2, self.frame_height // 2])
        # a 2x2 array to store the position of the red markers
        #red_pos = np.array([[0,0],[0,0]])
        self.start_time = time.time()
        #while True:
           

        #video.release()
        #cv2.destroyAllWindows()


    def camera_feed(self):
        while True:
            # Record the start time for calculating processing time
            start_time = time.time()

            # Read a frame from the webcam
            ret, frame = self.video.read()
            frame = cv2.resize(frame, [self.frame_width // 2, self.frame_height // 2])

            if not ret or frame is None:
                print("Failed to capture frame")
                break

            # Call track_black func and enhance black color in the frame and draw bounding boxes
            track_frame = self.track_black(frame)  # Create a copy to preserve the original frame

            # Call track_blue func and enhance blue color in the frame and draw bounding boxes
            track_frame = self.track_blue(frame)

            track_frame = self.track_red(frame)
            # Call track_green func and enhance green color in the frame and draw bounding boxes
            track_frame = self.track_green(frame)

            if self.camera_blocked(frame):
                print("Camera is blocked")

            cv2.imshow('Object Tracking', track_frame)
            
            # Break the loop if the user presses 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.video.release()
        cv2.destroyAllWindows()
"""
if __name__ == "__main__":
    tracker = ObjectTracker()
    tracker.main()
"""