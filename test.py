import numpy as np
from barriers import def_barriers

# positions = []
# ziele = []
# for i in range(6):
#     if i < 3:
#         x_start = 0.3 + i * 0.7
#         x_ziel = x_start 
#         y_start = 3.7
#         y_ziel = 0.3
#     else:
#         x_start = 0.3 + (i - 3) * 0.7
#         x_ziel = x_start
#         y_start = 3.2
#         y_ziel = 0.8
#     positions.append((x_start, y_start))
#     ziele.append((x_ziel, y_ziel))

# positions = np.array(positions)
# np.random.shuffle(ziele)  # Zufällige Zuordnung der Ziele
# ziele = np.array(ziele)

# print(positions)
# print(ziele)

# # room dimensions
# dim_x = 2.0  # Meter
# dim_y = 4.0   # Meter
# room_corners = np.array([[0, 0], [dim_x, 0], [dim_x, dim_y], [0, dim_y]])
# # obstacle corners
# obstacle_corners = np.array([[0.75, 1.95], [1.25, 1.95], [1.25, 2.05], [0.75, 2.05]])
# # step size for gridmap
# step_size = 0.05 # m adjustable kommt aus main.py "dx"



# def get_walls(corners, step_size):
#     walls = []
#     num_corners = len(corners)
#     for i in range(num_corners):
#         start = corners[i]
#         end = corners[(i + 1) % num_corners]  # Wrap around to the first corner
#         if start[0] == end[0]:  # Vertical wall (same x-coordinate)
#             y_values = np.arange(min(start[1], end[1]), max(start[1], end[1]) + step_size, step_size)
#             wall = np.column_stack((np.full_like(y_values, start[0]), y_values))
#         elif start[1] == end[1]:  # Horizontal wall, same y-coordinate
#             x_values = np.arange(min(start[0], end[0]), max(start[0], end[0]) + step_size, step_size)
#             wall = np.column_stack((x_values, np.full_like(x_values, start[1])))
#         else:
#             raise ValueError("Walls must be either vertical or horizontal. Please rearrange corners.")
#         walls.append(wall)
#     walls = np.vstack(walls)
#     walls = np.round(walls, decimals = 3) # avoid floating point issues
#     walls = np.unique(walls, axis=0)  # remove duplicate corner points
#     return walls

# # Hinderniskoordinaten der Eckpunkte

# # unten links, unten rechts, oben rechts, oben links
# neighbour_positions = [[0.5, 0.5], [1.5, 0.5], [0.5, 3.5], [1.5, 3.5], [1.0, 2.0]]
# robot_position = [1.0, 0.3]
# radius = 0.8
# encumbrance_barriers_integer = 0.1

# barriers_unfiltered = def_barriers(step_size)


# if not isinstance(barriers_unfiltered, np.ndarray):
#     barriers_unfiltered = np.array(barriers_unfiltered)
# if not isinstance(robot_position, np.ndarray):
#     robot_position = np.array(robot_position)

# dist = np.linalg.norm(barriers_unfiltered - robot_position, axis=1)
# valid_indices = np.where(dist <= radius)

# barrier_positions = barriers_unfiltered[valid_indices].tolist()
# encumbrance_barriers = [encumbrance_barriers_integer for i in range(len(barrier_positions))]


# print(f'barrier_positions: {barrier_positions} and length: {len(barrier_positions)}')
# print(f'encumbrance_barriers: {encumbrance_barriers} and length: {len(encumbrance_barriers)}')

# walls_function = get_walls(obstacle_corners, step_size)

# print(f'walls_function: {len(walls_function)}')

# print(f"walls_function: {walls_function}")

cell = [[0.0 , 1.2], [0.05, 1.25], [0.1, 1.3], [0.15, 1.35], [0.2, 1.4], [0.25, 1.45]]

x_cell = [point[0] for point in cell]
y_cell = [point[1] for point in cell]

goal_position = [1.0, 3.5]

x_cell = np.array(x_cell)
y_cell = np.array(y_cell)

print(f'x_cell: {x_cell}')
print(f'y_cell: {y_cell}')

goal_x = np.full_like(x_cell, goal_position[0])
goal_y = np.full_like(y_cell, goal_position[1])

print(f'goal_x: {goal_x}')
print(f'goal_y: {goal_y}')

tmp = np.column_stack((x_cell - goal_position[0], y_cell - goal_position[1]))

dists_to_goal = np.linalg.norm(tmp, axis=1)

print(f'dists_to_goal: {dists_to_goal}')


