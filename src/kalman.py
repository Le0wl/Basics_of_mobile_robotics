# Kalman module for state estimation of the robot

# This filter takes in the values from the camera
# Values taken are:
#   x_measured, y_measured, angle_prev, vel, angle_vel, cam_blocked
# Values calcualted and fed back in
#   x_est_prev, P_est_prev

import numpy as np
import math
import matplotlib.pyplot as plt

class Kalman:
    def __init__(self):
        self.x_est_prev = np.array([[0], [0], [0]])
        self.P_est_prev = np.diag([1, 1, 1])



    def kalman_filter(self,x_measured, y_measured, angle_prev, vel, angle_vel,cam_blocked = False):
        # --------------------Iniitialisation------------------
        # Set the sampling time
        Ts = 0.1

        # State transition matrix
        A = np.array([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])
                    
        angle_prev= np.deg2rad(angle_prev)
        # Control matrix
        B = np.array([[math.cos(angle_prev)*Ts, 0],
                    [-math.sin(angle_prev)*Ts, 0],
                    [0, Ts]])
        
        # Measurement matrix that relates to state vector to measurement
        H =np.array([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])

        # Process noise covariance matrix
        Q = np.diag([1, 1, 1])   # ADJUST ACCORDINGLY

        # Set R (Measurement noise covariance matrix) to infinity if the camera is blocked
        # Measurement noise covariance matrix
        if cam_blocked:
            R = np.diag([np.inf, np.inf, np.inf])
        else:
            # Set a regular value for R when the camera is not blocked
            R = np.diag([0.3, 0.3, 0.3])       # ADJUST ACCORDINGLY

        # Prediction step
        # Assuming dvx and dvy are control inputs
        U_in = np.array([[vel],
                        [-angle_vel]])

        # Prediction step
        x_est_a_priori = np.dot(A, self.x_est_prev) + np.dot(B, U_in)
        P_est_a_priori = np.dot(A, np.dot(self.P_est_prev, A.T)) + Q

        angle_prev = np.rad2deg(angle_prev)
        # Update step
        y = np.array([[x_measured], [y_measured], [angle_prev]])
        y_measured_pred = np.dot(H, x_est_a_priori)

        # Inovation" refers to the difference between the observed (or measured) data and the predicted

        # Check if the measured values are both 0
        if x_measured == 0 and y_measured == 0:
            # If both measured values are 0, set the innovation to 0
            innovation = np.zeros((3, 1))
        else:
            # For non-zero measured values, compute the innovation as usual
            innovation = y - y_measured_pred
        
        # prediction covariance measurement
        S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R
        
        # Kalman gain (tells how much the predictions should be corrected based on the measurements)
        K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))

        x_est = x_est_a_priori + np.dot(K, innovation)
        P_est = P_est_a_priori - np.dot(K, np.dot(H, P_est_a_priori))
        self.x_est_prev = x_est
        self.P_est_prev = P_est
        #x_est = [x_measured, y_measured, angle_prev]
        #transpose x_est
        #x_est = np.transpose(x_est)

        return x_est, P_est
