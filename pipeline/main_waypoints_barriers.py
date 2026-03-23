import numpy as np
import matplotlib.pyplot as plt
from vision import vision_pipeline
import planner
import kinematics
import cv2  


def add_artificial_barriers(map_grid):
    modified_map = map_grid.copy()
    
    b1_start = (int(round(448.2)), int(round(221.3)))
    b1_end = (int(round(445.1)), int(round(209.0)))
    cv2.line(modified_map, b1_start, b1_end, color=0, thickness=3)
    
    b2_start = (int(round(179.0)), int(round(184.4)))
    b2_end = (int(round(176.3)), int(round(165.9)))
    cv2.line(modified_map, b2_start, b2_end, color=0, thickness=3)
    
    return modified_map


original_road_mask, start_coordinates, end_coordinates = vision_pipeline(filename="CW2Map.jpg", folder="../data")

# Inject the obstacles into the map!
road_mask = add_artificial_barriers(original_road_mask)

print(f"Start coordinates (red): {start_coordinates}")
print(f"End coordinates (green): {end_coordinates}") 

manual_waypoints = [
    (516.3, 209.4),
    (357.1, 229.4),
    (339.8, 225.0),
    (333.7, 183.2),
    (328.7, 172.7),
    (311.4, 166.0),
    (301.4, 160.4),
    (275.8, 160.4),
    (240.1, 169.9),
    (128.2, 179.3)
]

all_waypoints = [start_coordinates] + manual_waypoints + [end_coordinates]

print(f"{len(all_waypoints)} total waypoints.")

full_pruned_path = []

for i in range(len(all_waypoints) - 1):
    start_pt = (int(round(all_waypoints[i][0])), int(round(all_waypoints[i][1]))) #round as dijkstra needs integers
    end_pt = (int(round(all_waypoints[i+1][0])), int(round(all_waypoints[i+1][1])))
    
    raw_segment = planner.dijkstra(road_mask, start_pt, end_pt)
    
    if not raw_segment:
        print(f"Blocked between {start_pt} and {end_pt}")
        exit()
        
    pruned_segment = planner.prune_path(road_mask,raw_segment)
    
    if i == 0:
        full_pruned_path.extend(pruned_segment)
    else:
        full_pruned_path.extend(pruned_segment[1:])

print(f"{len(full_pruned_path)} total driving points.")


current_x = float(start_coordinates[0])
current_y = float(start_coordinates[1])
current_theta = 0.0  
dt = 0.1

history_x = [current_x]
history_y = [current_y]

for wp_x, wp_y in full_pruned_path[1:]:
    distance_to_target = np.hypot(wp_x - current_x, wp_y - current_y)
    
    while distance_to_target > 0.5: #0.5 threshold
        vx, vy, omega = kinematics.get_velocities(
            current_x, current_y, current_theta, wp_x, wp_y
        )
        
        phi_1, phi_2 = kinematics.inverse_kinematics_global(
            vx, vy, omega, current_theta
        )
        
        actual_vx, actual_vy, actual_omega = kinematics.forward_kinematics(
            phi_1, phi_2, current_theta
        )
        
        current_x, current_y, current_theta = kinematics.update_position(
            current_x, current_y, current_theta, actual_vx, actual_vy, actual_omega, dt
        )
        
        history_x.append(current_x)
        history_y.append(current_y)

        distance_to_target = np.hypot(wp_x - current_x, wp_y - current_y)


plt.figure(figsize=(12, 12))
plt.imshow(road_mask, cmap='gray')


plt.scatter(history_x, history_y, c='red', s=5, label='Robot Trajectory')

#target waypoints
wp_x_list = [p[0] for p in all_waypoints]
wp_y_list = [p[1] for p in all_waypoints] 
plt.scatter(wp_x_list, wp_y_list, c='yellow', marker='o', s=60, edgecolors='black', label='Given Waypoints')

plt.scatter(start_coordinates[0], start_coordinates[1], c='lime', marker='o', s=120, edgecolors='black', label='Start')
plt.scatter(end_coordinates[0], end_coordinates[1], c='fuchsia', marker='X', s=120, edgecolors='black', label='End')

plt.title("Dynamic Obstacle Avoidance Trajectory")
plt.legend()
plt.axis('off') 
plt.show()