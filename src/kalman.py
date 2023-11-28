# Kalman module for state estimation of the robot
import numpy as np
import math

class Kalman:
    def kalman_filter(x_measured, y_measured, vx_measured, vy_measured, x_est_prev, P_est_prev, dvx=0, dvy=0, cam_blocked = False):
    
    # Combined initialization and Kalman filter algorithm for a 2D position and velocity estimation.

    # Parameters:
    # - x_measured: Measured x coordinate.
    # - y_measured: Measured y coordinate.
    # - vx_measured: Measured x velocity.
    # - vy_measured: Measured y velocity.

    # Returns:
    # - x_est: A posteriori state estimate.
    # - P_est: A posteriori state covariance matrix.
    
    # --------------------Iniitialisation------------------
    # Set the sampling time
    T_s = 0.1

    # State transition matrix
    A = np.array([[1, 0, T_s, 0], [0, 1, 0, T_s], [0, 0, 1, 0], [0, 0, 0, 1]]) 
    # Control matrix
    B = np.array([[T_s, 0], [0, T_s], [1, 0], [0, 1]])
    # Measurement matrix that relates to state vector to measurement
    H = np.eye(4)
    # Process noise covariance matrix
    Q = np.diag([0.1, 0.1, 0.1, 0.1])   # ADJUST ACCORDINGLY

    # Set R (Measurement noise covariance matrix) to infinity if the camera is blocked
    # Measurement noise covariance matrix
    if cam_blocked:
        R = np.diag([np.inf, np.inf, np.inf, np.inf])
    else:
        # Set a regular value for R when the camera is not blocked
        R = np.diag([0.3, 0.3, 0.3, 0.3])       # ADJUST ACCORDINGLY

    # Prediction step
    # Assuming dvx and dvy are control inputs
    U_in = np.array([[dvx], [dvy]])

    # Prediction step
    x_est_a_priori = np.dot(A, x_est_prev) + np.dot(B, U_in)
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T)) + Q

    # Update step
    y = np.array([[x_measured], [y_measured], [vx_measured], [vy_measured]])
    y_measured_pred = np.dot(H, x_est_a_priori)

    # Inovation" refers to the difference between the observed (or measured) data and the predicted
    innovation = y - y_measured_pred
    
    # prediction covariance measurement
    S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R
    
    # Kalman gain (tells how much the predictions should be corrected based on the measurements)
    K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))

    x_est = x_est_a_priori + np.dot(K, innovation)
    P_est = P_est_a_priori - np.dot(K, np.dot(H, P_est_a_priori))

    return x_est, P_est
