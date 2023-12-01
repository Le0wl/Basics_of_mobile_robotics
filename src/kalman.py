# Kalman module for state estimation of the robot
import numpy as np
import math
import matplotlib.pyplot as plt

class Kalman:
    def pos_vel_meas(x_est_prev, P_est_prev, motor_left_speed, motor_right_speed, thymio_angle, thymio_pos):
        cam_blocked = False

        # Speed of Thymio
        speed = (motor_left_speed + motor_right_speed) / 2 # Assuming motor_left_speed and motor_right_speed are functions

        # Initialize the X and Y values
        x_measured = 0
        y_measured = 0
        vx_measured = 0
        vy_measured = 0

        # Update X and Y position using camera info
        if not cam_blocked:
            x_measured = thymio_pos[0]
            y_measured = thymio_pos[1]
            vx_measured = speed * math.cos(thymio_angle)
            vy_measured = -speed * math.sin(thymio_angle)
        else:
            cam_blocked = True

        # Calculate dvx and dvy
        x_prev, y_prev, vx_prev, vy_prev = x_est_prev.flatten()

        dvx = speed * math.cos(thymio_angle) - vx_prev
        dvy = -speed * math.sin(thymio_angle) - vy_prev

        # Kalman filter prediction and update
        # Assuming x_est_prev and P_est_prev are your previous state estimate and covariance
        x_est, P_est = kalman_filter(x_measured, y_measured, vx_measured, vy_measured, x_est_prev, P_est_prev, dvx, dvy, cam_blocked)

        return x_est, P_est, cam_blocked

    def kalman_filter(x_measured, y_measured, vx_measured, vy_measured, x_est_prev, P_est_prev, dvx=0, dvy=0, cam_blocked = False):

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
    
# Initial state estimate
x_est_prev = np.array([[0], [0], [0], [0]])
# Initial state covariance matrix
P_est_prev = np.diag([1.0, 1.0, 1.0, 1.0])
# Set initial conditions
x_true = np.array([[0.0], [0.0], [1.0], [0.0]], dtype=float)  # True initial state
dt = 0.1  # Time step

# Lists to store true and estimated states
true_states = []
est_states = []

# Simulation loop
for _ in range(100):
    # True dynamics (in this case, a simple constant velocity motion model)
    x_true[0] += x_true[2] * dt
    x_true[1] += x_true[3] * dt

    # Simulate noisy measurements
    noise = np.random.normal(0, 0.1, size=(4, 1))
    measurements = x_true + noise

    # Run the Kalman filter
    x_est, P_est, _ = pos_vel_meas(x_est_prev, P_est_prev, x_true[2], x_true[3], 0.0, measurements.flatten())

    # Store results
    true_states.append(x_true.copy())
    est_states.append(x_est.copy())

    # Update previous state for the next iteration
    x_est_prev = x_est
    P_est_prev = P_est

# Convert lists to arrays for plotting
true_states = np.array(true_states)
est_states = np.array(est_states)

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(true_states[:, 0], true_states[:, 1], label='True Position', color='blue')
plt.plot(est_states[:, 0], est_states[:, 1], label='Estimated Position', color='red', linestyle='dashed')
plt.title('Kalman Filter Simulation')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.legend()
plt.show()

