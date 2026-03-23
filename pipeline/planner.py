import numpy as np
import heapq

def dijkstra(map_grid, start_xy, goal_xy):
    rows, cols = map_grid.shape

    queue = [(0, start_xy[0], start_xy[1])]
    costs = {start_xy: 0}
    came_from = {start_xy: None}

    neighbors = [(0, -1, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (1, 0, 1.0), (-1, -1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (1, 1, 1.414)] #1.414 for sqrt(2) for diagonal

    while queue:
        cost_current, cx, cy = heapq.heappop(queue)
        current = (cx, cy)

        if current == goal_xy:
            break

        for dx, dy, move_cost in neighbors:
            nx, ny = cx + dx, cy + dy
            next_node = (nx, ny)

            if 0 <= ny < rows and 0 <= nx < cols and map_grid[ny, nx] > 0:
                new_cost = cost_current + move_cost

                if next_node not in costs or new_cost < costs[next_node]:
                    costs[next_node] = new_cost
                    heapq.heappush(queue, (new_cost, nx, ny))
                    came_from[next_node] = current

    if goal_xy not in came_from:
        return []
    
    path = []
    current = goal_xy
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def line_of_sight(map_grid, p1, p2):
    x0, y0 = p1 
    x1, y1 = p2

    distance = int(np.hypot(x1 - x0, y1 - y0)) #need int because of discrete steps
    if distance == 0: 
        return True
    
    for j in range(distance + 1):
        t=j/distance 
        x = int(round(x0 + t * (x1 - x0)))# again need to round and int for discrete grid 
        y = int(round(y0 + t * (y1 - y0)))

        if map_grid[y, x] == 0:
            return False
    
    return True

def prune_path(map_grid, path): #switch to raycast pruning, basic pruning only gave zigzag 
    if len(path) < 3:
        return path
    
    pruned = [path[0]]
    index = 0

    while index < len(path) - 1:
        furthest_visible_pixel = index + 1

        for i in range(len(path)-1, index, -1):
            if line_of_sight(map_grid, path[index], path[i]):
                furthest_visible_pixel = i
                break
        
        pruned.append(path[furthest_visible_pixel])
        index = furthest_visible_pixel

    pruned.append(path[-1])
    return pruned