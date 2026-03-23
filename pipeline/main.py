import numpy as np
import matplotlib.pyplot as plt
from vision import vision_pipeline
import planner
import kinematics


road_mask, start_coordinates, end_coordinates = vision_pipeline(filename="CW2Map.jpg", folder="../data")

print(f"Start coordinates (red): {start_coordinates}")
print(f"End coordinates (green): {end_coordinates}")   

raw_path = planner.dijkstra(road_mask, start_coordinates, end_coordinates)

if not raw_path:
    print("Error: No path found.")
else:
    pruned_path = planner.prune_path(road_mask,raw_path)
    print(f"{len(raw_path)} raw pixels pruned down to {len(pruned_path)} waypoints.")


    current_x = float(start_coordinates[0])
    current_y = float(start_coordinates[1])
    current_theta = np.pi  # Assume it starts facing 
    dt = 0.1

    history_x = [current_x]
    history_y = [current_y]

    for wp_x, wp_y in pruned_path[1:]:
        
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
    waypoint_x = [p[0] for p in pruned_path]
    waypoint_y = [p[1] for p in pruned_path]
    plt.scatter(waypoint_x, waypoint_y, c='blue', marker='o', s=60, label='Pruned Waypoints')
    
    plt.scatter(start_coordinates[0], start_coordinates[1], c='lime', marker='o', s=100, label='Start')
    plt.scatter(end_coordinates[0], end_coordinates[1], c='fuchsia', marker='X', s=100, label='End')
    
    plt.title("Full Autonomous Navigation Pipeline")
    plt.legend()
    plt.axis('off') 
    plt.show()
