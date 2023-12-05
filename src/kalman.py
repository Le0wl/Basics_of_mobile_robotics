# Kalman module for state estimation of the robot
import numpy as np
import math
import matplotlib.pyplot as plt

class Kalman:
    def kalman_filter(x_measured, y_measured, vx_measured, vy_measured, x_est_prev, P_est_prev, dvx=0, dvy=0, cam_blocked = False):

        # --------------------Iniitialisation------------------
        # Set the sampling time
        T_s = 0.5

        # State transition matrix
        A = np.array([[1, 0, T_s, 0],
                    [0, 1, 0, T_s],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
        # Control matrix
        B = np.array([[T_s, 0],
                    [0, T_s],
                    [1, 0],
                    [0, 1]])
        # Measurement matrix that relates to state vector to measurement
        H = np.eye(4)
        # Process noise covariance matrix
        Q = np.diag([2, 2, 2, 2])   # ADJUST ACCORDINGLY

        # Set R (Measurement noise covariance matrix) to infinity if the camera is blocked
        # Measurement noise covariance matrix
        if cam_blocked:
            R = np.diag([np.inf, np.inf, np.inf, np.inf])
        else:
            # Set a regular value for R when the camera is not blocked
            R = np.diag([0.3, 0.3, 0.3, 0.3])       # ADJUST ACCORDINGLY

        # Prediction step
        # Assuming dvx and dvy are control inputs
        U_in = np.array([dvx, dvy])

        # Prediction step
        x_est_prev = np.squeeze(x_est_prev)
        x_est_a_priori = np.dot(A, x_est_prev) + np.dot(B, U_in)
        P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T)) + Q

        # Update step
        y = np.array([[x_measured], [y_measured], [vx_measured], [vy_measured]])
        y_measured_pred = np.dot(H, x_est_a_priori)

        # Inovation" refers to the difference between the observed (or measured) data and the predicted

        # Check if the measured values are both 0
        if x_measured == 0 and y_measured == 0:
            # If both measured values are 0, set the innovation to 0
            innovation = np.zeros((4, 1))
        else:
            # For non-zero measured values, compute the innovation as usual
            innovation = y - y_measured_pred
        
        # prediction covariance measurement
        S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R
        
        # Kalman gain (tells how much the predictions should be corrected based on the measurements)
        K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))

        x_est = x_est_a_priori + np.dot(K, innovation)
        P_est = P_est_a_priori - np.dot(K, np.dot(H, P_est_a_priori))

        return x_est, P_est

    # Initial and final positions
    start_point = np.array([0, 0])
    end_point = np.array([5000, 5000])

    # Calculate the total distance
    total_distance = np.linalg.norm(end_point - start_point)

    # Define the time step
    time_step = 100

    # Calculate the number of steps
    num_steps = int(total_distance / time_step)

    # Calculate the step vector
    step_vector = (end_point - start_point) / num_steps

    # Simulation parameters
    total_time_steps = 100

    # Initialize parameters 
    x_est = 0, 0, 0, 0
    x_est_prev = [0 , 0 , 0 , 0]
    P_est_prev = 0

    # True initial state
    P_est = np.eye(4)
    dvx = 0
    dvy = 0

    # Required initialization
    cam_blocked = True
    motor_left_speed = 200
    motor_right_speed = 200
    readimg_thymio_pos = [0, 0]
    readimg_thymio_angle_deg = 45
    readimg_thymio_angle = math.radians(readimg_thymio_angle_deg)

    # Calculate the speed with left and right speed
    speed = (motor_left_speed + motor_right_speed) / 2

    # Lists to store simulation results
    filtered_states = [x_est]
    current_point_append = []

    # Initialize current_point
    current_point = start_point.astype(float).copy()  # Explicitly cast to float
    filtered_x = []
    filtered_y = []

    # Simulation loop
    for step in range(num_steps + 1):

        # Initialization
        x_measured, y_measured, vx_measured, vy_measured = 0, 0, 0, 0

        # Plot the actual incremental steps
        # plt.plot(int(current_point[0]), int(current_point[1]), 'go', label=f'Step {step}')  # Explicitly cast to int
        current_point += step_vector
        readimg_thymio_pos = current_point
            
        # Append current_point to the list
        current_point_append.append(current_point.copy())

        # Calculate dvx and dvy
        x_prev, y_prev, vx_prev, vy_prev = x_est_prev[0], x_est_prev[1], x_est_prev[2], x_est_prev[3]

        # Calculate dvx and dvy using radians
        dvx = speed * math.cos(readimg_thymio_angle) - vx_prev
        dvy = -speed * math.sin(readimg_thymio_angle) - vy_prev

        # Update true state using motion model (you need to implement motion_model function)
        if not cam_blocked:
            x_measured, y_measured = readimg_thymio_pos[0], readimg_thymio_pos[1]
            vx_measured, vy_measured = speed * math.cos(readimg_thymio_angle), -speed * math.sin(readimg_thymio_angle)

        # Update estimates using Kalman filter (you need to implement kalman_filter function)
        x_est, P_est = kalman_filter(x_measured, y_measured, vx_measured, vy_measured, x_est_prev, P_est_prev, dvx, dvy, cam_blocked)
        
        # to keep only 1 line of the filtered states
        x_est_store = x_est[:, 0].reshape(-1, 1)
        x_est_prev = x_est[:, 0].reshape(-1, 1)
        P_est_prev = P_est

        filtered_states.append(x_est_store)

        # Extract x and y coordinates from filtered_states
        filtered_x.append(x_est_store[0, 0])  # Assuming x_est is a 4x1 matrix
        filtered_y.append(x_est_store[1, 0])  # Assuming x_est is a 4x1 matrix

    # Extract x and y coordinates for plotting
    true_x = np.array(current_point_append)[:, 0]
    true_y = np.array(current_point_append)[:, 1]

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(true_x, true_y, label='True Path', marker='o')
    plt.plot(filtered_x, filtered_y, label='Filtered Path', marker='x')
    plt.title('Kalman Filter Simulation')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.grid(True)
    plt.show()