# zu übergebende Parameter:
# Raumdimensionen
# Hindernisdimensionen und Position
# Roboterdimensionen
# aktuelle Roboterpositionen

# update barriers

# dimensionen der barriers (step_size: 0.01m) (=1cm)
import numpy as np
# from bluerov_simulation import bluerov_simulation # avoid circular import


class barriers:

    def __init__(self, step_size):
        self.step_size = step_size  # m adjustable kommt aus main.py "dx"
        self.lim_basin_x = np.array([0.0, 2.0])
        self.lim_basin_y = np.array([0.0, 4.0])
        self.lim_obs_x = np.array([0.75, 1.25])
        self.lim_obs_y = np.array([1.95, 2.05])

    def get_walls(self, corners):
        walls = []
        num_corners = len(corners)
        for i in range(num_corners):
            start = corners[i]
            end = corners[(i + 1) %
                          num_corners]  # Wrap around to the first corner
            if start[0] == end[0]:  # Vertical wall (same x-coordinate)
                y_values = np.arange(min(start[1], end[1]),
                                     max(start[1], end[1]) + self.step_size,
                                     self.step_size)
                wall = np.column_stack((np.full_like(y_values,
                                                     start[0]), y_values))
            elif start[1] == end[1]:  # Horizontal wall, same y-coordinate
                x_values = np.arange(min(start[0], end[0]),
                                     max(start[0], end[0]) + self.step_size,
                                     self.step_size)
                wall = np.column_stack(
                    (x_values, np.full_like(x_values, start[1])))
            else:
                raise ValueError(
                    "Walls must be either vertical or horizontal. Please rearrange corners."
                )
            walls.append(wall)
        walls = np.vstack(walls)
        walls = np.round(walls, decimals=3)  # avoid floating point issues
        walls = np.unique(walls, axis=0)  # remove duplicate corner points
        return walls

    def def_barriers(self):

        basin_corners = np.array([[self.lim_basin_x[0], self.lim_basin_y[0]],
                                  [self.lim_basin_x[1], self.lim_basin_y[0]],
                                  [self.lim_basin_x[1], self.lim_basin_y[1]],
                                  [self.lim_basin_x[0], self.lim_basin_y[1]]])
        obstacle_corners = np.array([[self.lim_obs_x[0], self.lim_obs_y[0]],
                                     [self.lim_obs_x[1], self.lim_obs_y[0]],
                                     [self.lim_obs_x[1], self.lim_obs_y[1]],
                                     [self.lim_obs_x[0], self.lim_obs_y[1]]])

        obstacle_walls = self.get_walls(obstacle_corners)
        basin_walls = self.get_walls(basin_corners)

        return np.vstack((obstacle_walls, basin_walls))

    def get_limits(self):
        # return basinlimits, obstacle limits

        return np.array([self.lim_basin_x, self.lim_basin_y
                         ]), np.array([self.lim_obs_x, self.lim_obs_y])

    # # Hindernis in der barriers eintragen
    # hindernis_x = (hindernis1[:, 0] / self.step_size).astype(int)
    # hindernis_y = (hindernis1[:, 1] / self.step_size).astype(int)

    # barriers = np.ones((int(dim_x / self.step_size), int(dim_y / self.step_size))) # 1 = mission space
    #     # np.ones((200, 400))  # 200 x 400 = 2m x 4m mit step_size 0.01m

    # barriers[hindernis_x[0]:hindernis_x[2], hindernis_y[0]:hindernis_y[2]] = np.nan  # nan = Hindernis
    # # barriers[75:125, 195:204] = np.nan  # Hindernis in der barriers eintragen

    # # RÄNDER in der barriers als Hindernis markieren
    # barriers[0, :] = np.nan  #  Wand x=0
    # barriers[-1, :] = np.nan  #  Wand x=dim_x
    # barriers[:, 0] = np.nan  #  Wand y=0
    # barriers[:, -1] = np.nan  #  Wand y=dim_y

    # return barriers


# def euc_to_grid(self, position):
#     # position in meter (x,y)
#     # position in barriers umrechnen
#     grid_x = int(position[0] / self.step_size)
#     grid_y = int(position[1] / self.step_size)
#     return (grid_x, grid_y)

# def update_barriers(self, barriers, grid_position_rob1, grid_position_rob2):
#     # roboterposition bestehend aus x und y in  meter
#     # Roboterpositionen in barriers-Indizes umrechnen):

#     #ggf unnötig

# return barriers
