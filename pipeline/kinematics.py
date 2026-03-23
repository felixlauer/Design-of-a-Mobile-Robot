import matplotlib.pyplot as plt
import numpy as np

L = 1.25 #half of the length of the wheelbase which is 2.5 pixels
r = 0.25 #radius of the wheel which is 0.5 pixels diameter

inverse_kinematics_matrix = np.array([[1/r, 0, L/r], [1/r, 0, -L/r]])

def inverse_kinematics_global(vx, vy, omega, theta):
    I_P_dot = np.array([[vx], [vy], [omega]])

    R_I_to_0 = np.array([[np.cos(theta), np.sin(theta), 0], [-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

    local_velocity = np.dot(R_I_to_0, I_P_dot)

    wheel_velocities = np.dot(inverse_kinematics_matrix, local_velocity)

    phi_1 = wheel_velocities[0][0]
    phi_2 = wheel_velocities[1][0]
    return phi_1, phi_2

def forward_kinematics(phi_1, phi_2, theta):
    phi_dot = np.array([[phi_1], [phi_2]])

    J_forward = np.array([[r/2, r/2], [0, 0], [r/(2*L), -r/(2*L)]])

    R_0_to_I = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

    local_velocity = np.dot(J_forward, phi_dot)
    global_velocity = np.dot(R_0_to_I, local_velocity)

    vx = global_velocity[0][0]
    vy = global_velocity[1][0]
    omega = global_velocity[2][0]

    return vx, vy, omega

def update_position(x, y, theta, vx, vy, omega, dt=0.1):
    x_new = x + (vx * dt)
    y_new = y + (vy * dt)
    theta_new = theta + (omega * dt)

    theta_new = (theta_new + np.pi) % (2 * np.pi) - np.pi

    return x_new, y_new, theta_new

def get_velocities(current_x, current_y, current_theta, target_x, target_y):
    dx = target_x - current_x
    dy = target_y - current_y
    distance = np.hypot(dx, dy)
    target_theta = np.arctan2(dy, dx)

    angle_diff = target_theta - current_theta
    angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

    Kp_turn = 8.0 #2 was too less -> huge arcs -> go to 4
    Kp_drive = 1.0 

    omega = Kp_turn * angle_diff

    local_v = Kp_drive * distance *np.cos(angle_diff) #add this to slow then when large angle difference
    local_v = max(0.0, min(local_v, 15.0)) # Introduce speed limit

    global_vx = local_v * np.cos(current_theta)
    global_vy = local_v * np.sin(current_theta)
    return global_vx, global_vy, omega
