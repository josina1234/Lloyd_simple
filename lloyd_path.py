# Übergabeparamter:
# - Start- und Zielpositionen der Roboter
# - Raum- und Hindernisabmessungen
# - Roboterabmessungen
# roboter1_start = (0.5, 0.5)
# roboter1_ziel = (1.5, 3.5)
# roboter2_start = (1.5, 0.5)
# roboter2_ziel = (0.5, 3.5)
# barriers

# roboter_radius = 0.25

# in diesem Skript definierte Parameter:
# Sicherheitsabstand zwischen Roboter und Hindernis
# sicherheitsabstand zwischen Robotern
# sicherheitsabstand Roboter und wand

# barriers in jeden Schritt aktualisieren

from matplotlib.pylab import beta
from barriers import def_barriers
import numpy as np
import math


class Lloyd:

    def __init__(self, robot_i_position, radius, encumbrance_i,
                 encumbrance_neighbours, k_p_i, step_size, dt,
                 encumbrance_barriers):  #ggf noch v_max TODO
        # initialisieren des Klassenobjektes des i-ten Roboters
        self.robot_position = robot_i_position  # zunächst Startpositionen
        self.radius = radius  # same for all robots
        self.encumbrance = encumbrance_i
        self.encumbrance_neighbours_unfiltered = encumbrance_neighbours
        self.k_p = k_p_i
        self.step_size = step_size
        self.dt = dt
        self.encumbrance_barriers_float = encumbrance_barriers
        self.barriers_unfiltered = def_barriers(step_size)  # TODO
        # ggf noch v_max TODO

    # wird in jeder Iteration aufgerufen und deklariert die Nachbarpositionen neu
    def aggregate(self, neighbour_positions, beta_i, goal_position_i):
        self.neighbour_positions = neighbour_positions
        self.beta = beta_i
        self.goal_position = goal_position_i
        self.filter_neighbours()
        self.filter_barriers()

    def filter_neighbours(self):
        # Nachbarn filtern, die innerhalb eines bestimmten Abstands sind (2*r) maximaler Sensing radius
        # ggf hier noch mal sicherstellen. dass neighbour_positions und robot_position ein numpy array sind
        if not isinstance(self.neighbour_positions, np.ndarray):
            self.neighbour_positions = np.array(self.neighbour_positions)
        if not isinstance(self.robot_position, np.ndarray):
            self.robot_position = np.array(self.robot_position)

        dist = np.linalg.norm(self.neighbour_positions - self.robot_position,
                              axis=1)
        valid_indices = np.where(dist <= 2 * self.radius)

        self.neighbour_positions = self.neighbour_positions[
            valid_indices].tolist()
        self.encumbrance_neighbours = [
            self.encumbrance_neighbours_unfiltered[i] for i in valid_indices
        ]

    def filter_barriers(self):
        # Hindernisse filtern, die innerhalb eines bestimmten Abstands sind 1*r
        # ggf hier noch mal sicherstellen. dass barriers und robot_position ein numpy array sind
        if not isinstance(self.barriers_unfiltered, np.ndarray):
            self.barriers_unfiltered = np.array(self.barriers_unfiltered)
        if not isinstance(self.robot_position, np.ndarray):
            self.robot_position = np.array(self.robot_position)

        dist = np.linalg.norm(self.barriers_unfiltered - self.robot_position,
                              axis=1)
        valid_indices = np.where(dist <= self.radius)

        self.barrier_positions = self.barriers_unfiltered[
            valid_indices].tolist()
        self.encumbrance_barriers = [
            self.encumbrance_barriers_float
            for i in range(len(self.barrier_positions))
        ]  # prüfen ob gewüschte form herauskommt

    def get_centroid(self):
        # neue Centroiden berechnen
        circle_points = self.get_circle_points()  # tupel-liste

        if len(self.neighbour_positions) > 0:
            # compute voronoi cell

            # get cell points ignoring any encumbrances and barriers
            # Case: delta_ij <= (||p_i - p_j||)/2
            # cell_points are all q within circle with ||q-p_i|| < ||q-p_j|| for all neighbours j
            cell_points = self.find_closest_points(circle_points)  # tupel-list
            # filter out points too close to barriers
            cell_points = self.consider_barriers(cell_points)
            # filter cell points considering encumbrances
            # Case: delta_ij > (||p_i - p_j||)/2
            #  ||q-p_i|| < ||q-p_j~|| for all neighbours j
            cell_points_filtered = self.consider_encumbrances(cell_points)
            # if no points left in cell_points_filtered, return current position as centroid
            if len(cell_points_filtered) == 0:
                cell_points_filtered = [self.robot_position]

            x_cell, y_cell = zip(
                *cell_points_filtered
            )  # unzip tupel-list (* is unpacking operator)

        else:
            # circle_points are voronoi cell
            x_cell, y_cell = zip(
                *circle_points)  # unzip tupel-list (* is unpacking operator)

        x_cell_no_neigh, y_cell_no_neigh = zip(
            *circle_points)  # unzip tupel-list (* is unpacking operator)

        # compute scalar values for weighted centroid
        scalar_values = self.compute_scalar_values(x_cell, y_cell)
        scalar_values_no_neigh = self.compute_scalar_values(
            x_cell_no_neigh, y_cell_no_neigh)

        # make sure everything is numpy array
        x_cell = np.array(x_cell)
        y_cell = np.array(y_cell)
        scalar_values = np.array(scalar_values)
        x_cell_no_neigh = np.array(x_cell_no_neigh)
        y_cell_no_neigh = np.array(y_cell_no_neigh)
        scalar_values_no_neigh = np.array(scalar_values_no_neigh)

        # compute weighted centroids

        c1_x = np.sum(x_cell * scalar_values) / np.sum(scalar_values)
        c1_y = np.sum(y_cell * scalar_values) / np.sum(scalar_values)
        c2_x = np.sum(x_cell_no_neigh *
                      scalar_values_no_neigh) / np.sum(scalar_values_no_neigh)
        c2_y = np.sum(y_cell_no_neigh *
                      scalar_values_no_neigh) / np.sum(scalar_values_no_neigh)

        c1 = np.array([c1_x, c1_y])  # nieghbours
        c2 = np.array([c2_x, c2_y])  # no neighbours

        return c1, c2

    def get_circle_points(self):
        x_center, y_center = self.robot_position

        x_min = x_center - self.radius
        x_max = x_center + self.radius
        y_min = y_center - self.radius
        y_max = y_center + self.radius

        x_coords = np.round(np.arange(x_min, x_max + self.step_size,
                                      self.step_size),
                            decimals=3)  # to avoid floating point issues
        y_coords = np.round(np.arange(y_min, y_max + self.step_size,
                                      self.step_size),
                            decimals=3)

        x, y = np.meshgrid(x_coords, y_coords)

        distances = np.sqrt((x - x_center)**2 + (y - y_center)**2)
        valid_indices = np.where(distances <= self.radius)

        circle_points = list(zip(x[valid_indices],
                                 y[valid_indices]))  # zip makes tuples
        return circle_points

    def find_closest_points(self, circle_points):
        # all circle_points closer to robot i than to any neighbour
        dists_to_robot = np.linalg.norm(np.array(circle_points) -
                                        self.robot_position,
                                        axis=1)
        dists_to_neighbours = np.linalg.norm(
            np.array(circle_points)[:, np.newaxis] -
            np.array(self.neighbour_positions),
            axis=2
        )  # shape (num_circle_points, num_neighbours) # table with all distances
        # demand: dist to robot < dist to any neighbour
        valid_indices = np.all(
            dists_to_robot[:, np.newaxis] < dists_to_neighbours,
            axis=1)  # shape (num_circle_points,) boolean array

        closer_points = np.array(circle_points)[valid_indices].tolist()
        return closer_points  # list of tuples

    def consider_encumbrances(self, cell_points):

        index = []
        for j, neighbour in enumerate(self.neighbour_positions):
            # encumbrance of neighbours
            # vector from i to j for middle point between i and j
            dx = self.robot_position[0] - neighbour[0]
            dy = self.robot_position[1] - neighbour[1]

            if abs(dx) < 0.001:
                dx = 0.001  # avoid division by zero
            if abs(dy) < 0.001:
                dy = 0.001  # avoid division by zero

            m = dy / dx  # slope

            if abs(m) < 0.001:
                m = 0.001  # avoid division by zero

            # coordinates of middle point
            xm = (self.robot_position[0] + neighbour[0]) / 2
            ym = (self.robot_position[1] + neighbour[1]) / 2

            # length of that vector

            dm = np.linalg.norm(xm - self.robot_position[0],
                                ym - self.robot_position[1])

            # r < 1/2 dm --> r < 1/4 ||p_i - p_j|| --> delta_ij < 1/2 ||p_i - p_j||

            if dm < self.encumbrance_neighbours[j] + self.encumbrance:
                normal_ij = np.array([dx, dy]) / np.linalg.norm([dx, dy])
                solx = xm + (self.encumbrance_neighbours[j] +
                             self.encumbrance - dm) * normal_ij[0]
                soly = ym + (self.encumbrance_neighbours[j] +
                             self.encumbrance - dm) * normal_ij[1]

                # Geradengleichung für Trennungslinie der Zelle aufstellen und umstellen
                # y = - 1/m * (x - solx) + soly
                # y + 1/m * (x - solx) - soly = 0
                if self.robot_position[1] + 1 / m * (self.robot_position[0] -
                                                     solx) - soly > 0:
                    # Robot i is above the line
                    for k, point in enumerate(cell_points):
                        if point[1] + 1 / m * (point[0] - solx) - soly < 0:
                            index.append(
                                k)  # all points on the wrong side of the line
                else:
                    # Robot i is below the line
                    for k, point in enumerate(cell_points):
                        if point[1] + 1 / m * (point[0] - solx) - soly > 0:
                            index.append(
                                k)  # all points on the wrong side of the line
        cell_points_filtered = [
            point for k, point in enumerate(cell_points) if k not in index
        ]
        return cell_points_filtered

    def consider_barriers(self, cell_points):
        # encumbrance of barriers
        # first we filter out points that are too close to barriers
        # remove coordinates in cell_points that are closer than encumbrance_barriers to any barrier
        if len(self.barrier_positions) > 0:
            barrier_positions = np.array(self.barrier_positions)
            dists_to_barriers = np.linalg.norm(
                np.array(cell_points)[:, np.newaxis] - barrier_positions,
                axis=2)
            valid_indices = np.all(dists_to_barriers
                                   > self.encumbrance_barriers,
                                   axis=1)
            cell_points = np.array(cell_points)[valid_indices].tolist()

        return cell_points

    def compute_scalar_values(self, x_cell, y_cell):
        x_cell = np.array(x_cell)
        y_cell = np.array(y_cell)

        dists_to_goal = np.linalg.norm(np.column_stack(
            (x_cell - self.goal_position[0], y_cell - self.goal_position[1])),
                                       axis=1)
        scalar_vals = np.exp(-dists_to_goal /
                             self.beta)  # beta is spreading factor rho
        return scalar_vals.tolist()

    def compute_control(self, **kwargs):
        centroid = kwargs.get('centroid', None)
        if centroid is None:
            centroid, _ = self.get_centroid()

        error = centroid - self.robot_position
        u = self.k_p * error  # control input # PROPORTIONAL CONTROLLER
        # TODO maybe upgrade to General COntrol Law
        # return u if np.linalg.norm(u) <= self.v_max else u / np.linalg.norm(u) * self.v_max TODO
        return u


def applyrules(i, params, beta, current_positions, c1, c2, theta,
               goal_positions, BlueRovs, c1_no_rotation, d2, d4):
    c1_i = np.array(c1[i])  # centroid with neighbours
    current_position_i = np.array(current_positions[i])

    # first condition
    dist_c1_c2 = np.linalg.norm(c1_i - np.array(c2[i]))
    if dist_c1_c2 > d2 and np.linalg.norm(current_position_i -
                                          c1_i) < params["d1"]:
        beta[i] = max(beta[i] - params["dt"], params["beta_min"])
    else:
        beta[i] = beta[i] - params["dt"] * (beta[i] - params["betaD"][i])

    # second condition
    dist_c1_c2_d4 = dist_c1_c2 > d4
    if dist_c1_c2_d4 and np.linalg.norm(current_position_i -
                                        c1_i) < params["d3"]:
        theta[i] = min(theta[i] + params["dt"], np.pi / 2)
    else:
        theta[i] = max(0, theta[i] - params["dt"])

    # third condition
    if theta[i] == np.pi / 2 and np.linalg.norm(current_position_i - np.array(
            c1_no_rotation[i])) > np.linalg.norm(current_position_i - c1_i):
        theta[i] = 0

    # compute the angle and new position
    angle = np.arctan2(goal_positions[i][1] - current_position_i[1],
                       goal_positions[i][0] - current_position_i[0])
    new_angle = angle - theta[i]
    distance = np.sqrt((goal_positions[i][0] - current_position_i[0])**2 +
                       (goal_positions[i][1] - current_position_i[1])**2)
    BlueRovs.goal_positions[i][
        0] = current_position_i[0] + distance * math.cos(new_angle) # new goalposition x
    BlueRovs.goal_positions[i][
        1] = current_position_i[1] + distance * math.sin(new_angle) # new goalposition y
    # BlueRovs.destinations[i][0] = current_position
