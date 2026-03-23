import matplotlib.pyplot as plt
from vision import vision_pipeline

road_mask, start_coords, end_coords = vision_pipeline(filename="CW2Map.jpg", folder="../data")

fig, ax = plt.subplots(figsize=(12, 12))
ax.imshow(road_mask, cmap='gray')
ax.set_title("INTERACTIVE WAYPOINT PICKER\nLeft-Click to place | Right-Click to remove | Middle-Click (or Enter) to finish")

ax.scatter(start_coords[0], start_coords[1], c='lime', marker='o', s=150, edgecolors='black', label='Start')
ax.scatter(end_coords[0], end_coords[1], c='fuchsia', marker='X', s=150, edgecolors='black', label='End')
ax.legend()

clicked_points = plt.ginput(n=-1, timeout=0)

plt.close()

print("manual_waypoints = [")
for pt in clicked_points:
    print(f"({pt[0]:.1f}, {pt[1]:.1f}),")
print("]")
