import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import copy
from BlueRovsInit import BlueRovsInit
from lloyd_path import Lloyd, applyrules
from barriers import def_barriers, get_limits


class bluerov_simulation:

    def __init__(self, parameters):
        self.params = parameters
        self.max_velocity = 0  # tracks max velocity for reporting
        if self.params["N"] > 6:
            print("Maximale Anzahl an Bluerovs ist 6 aufgrund der basingröße.")
            self.params["N"] = 6
            print(f'{self.params["N"]} Bluerovs werden verwendet.')

        self.c1 = np.zeros(
            (self.params["N"], 2))  # Initialization of  actual centroids
        self.c2 = np.zeros(
            (self.params["N"],
             2))  # Initialization of  centroids without neighbors
        self.c1_no_rotation = np.zeros(
            (self.params["N"],
             2))  # Initialization of  actual centroids without rotation
        self.c2_no_rotation = np.zeros(
            (self.params["N"],
             2))  # Initialization of  centroids without neighbors and rotation
        self.flag = np.zeros(
            self.params["N"]
        )  # Initialization of the flag that indicates if the robot i is in its goal region
        self.file_path = 'test' + str(
            self.params["h"]) + '.txt'  # File path to save the data
        self.theta = np.zeros(
            self.params["N"])  # initialization of current orientations
        self.flag_convergence = 0  # Initialization of the flag that indicates if all the robots have entered their goal regions
        self.current_positions = None
        self.goal_positions = None
        self.Lloyd = None
        """self.Lloyd2 = None  # without neighbors (virtual) ggf unnecessary, TODO"""
        self.BlueRovs = None  # BlueROV objects # will be initialized later
        self.beta = self.params["betaD"].copy(
        )  # Initialization of the spreading factor rho
        self.step = 0  # simulation step counter

        # Plotting components for efficiency
        self.fig = None
        self.ax1 = None
        self.robot_plots = []  # Store plot objects for efficient updates
        self.goal_plots = []
        self.trajectory_plots = []
        self.barriers_plotted = False
        self.trajectories = [[] for _ in range(self.params["N"])
                             ]  # Store trajectory history

    def initialize_simulation(self):
        # Start und Zielpositionen der N-Bluerovs
        self.BlueRovs = BlueRovsInit(self.params)

        self.current_positions = self.BlueRovs.start_positions
        self.goal_positions = copy.deepcopy(
            self.BlueRovs.goal_positions
        )  # changes in deepcopy do not affect original or vice versa

        # Initialisierung des Lloyd-Algorithmus
        self.Lloyd = [
            Lloyd(self.current_positions[i], self.params["radius"],
                  self.params["size"][i],
                  np.delete(self.params["size"], i, axis=0),
                  self.params["k"][i], self.params["dx"], self.params["dt"],
                  self.params["encumbrance_barriers"], self.params["v_max"][i])
            for i in range(self.params["N"])
        ]

        # Initialisierung des virtuellen Lloyd-Algorithmus ohne Nachbarn
        """self.Lloyd2 = [
            Lloyd(self.current_positions[i], self.params["radius"],
                  self.params["size"][i],
                  np.delete(self.params["size"], i, axis=0),
                  self.params["k"][i], self.params["dx"], self.params["dt"],
                  self.params["encumbrance_barriers"], self.params["v_max"][i])
            for i in range(self.params["N"])
        ]  # ggf unnecessary, Lloyd2 just for debugging purposes TODO"""

        # plotting setup with barriers and improved efficiency
        self._setup_plot_with_barriers()

    def simulate_step(self):
        for i in range(self.params["N"]):
            neighbour_positions = np.delete(
                self.current_positions, i,
                axis=0)  # removes the position of i-th robot

            # Update both Lloyd instances with Nachbarinformationen
            # Ergebnis sind zwei arrays mit den Nachbarpositionen bzw. der Größe der Nachbarn
            self.Lloyd[i].aggregate(neighbour_positions, self.beta[i],
                                    self.goal_positions[i])
            """self.Lloyd2[i].aggregate(neighbour_positions, self.beta[i], self.goal_positions[i])"""

            # Berechnung der neuen Zielpositionen (c1 und c2)
            # c1 mit Nachbarn
            # c2 ohne Nachbarn
            self.c1[i], self.c2[i] = self.Lloyd[i].get_centroid()
            """self.c1_no_rotation[i], _ = self.Lloyd2[i].get_centroid()"""

            # control input
            u = self.Lloyd[i].compute_control(
                centroid=[self.c1[i][0], self.c1[i][1]])

            if np.linalg.norm(u) > self.max_velocity:  # in erster iteration 0
                self.max_velocity = np.linalg.norm(u)

            d2 = 3 * max(self.params["size"])
            d4 = d2  # distance for repulsion from other robots # originally = d2
            # d2 = 0.8
            # d4 = 1.6

            applyrules(i, self.params, self.beta, self.current_positions,
                       self.c1, self.c2, self.theta, self.goal_positions,
                       self.BlueRovs, self.c1_no_rotation, d2, d4)

            if np.linalg.norm(self.current_positions[i] -
                              self.goal_positions[i]) < self.params["radius"]:
                self.flag[i] = 1
            else:
                self.flag[i] = 0

            if sum(self.flag) == self.params["N"]:
                self.flag_convergence += 1

            if sum(self.flag
                   ) == self.params["N"] and self.flag_convergence == 1:
                print(
                    f"travel time: {round(self.step * self.params['dt'], 3)}, (s). Max velocity: {round(self.max_velocity, 3)}, (m/s)"
                )

            if self.flag_convergence == self.params["waiting_time"] - 1:
                self.cleanup_plot()

            if self.params["write_file"] == 1:
                with open(self.file_path, 'a') as file:
                    # maybe add barriers to data
                    data = f"{self.step}, {i}, {self.current_positions[i][0]}, {self.current_positions[i][1]}, {self.goal_positions[i][0]}, {self.goal_positions[i][1]}, {self.beta[i]}, {self.params['size'][i]}, {self.c1[i][0]}, {self.c1[i][1]}, {self.params['k'][i]}, {self.params['dt']}\n"
                    file.write(data)

            self.current_positions[i][0], self.current_positions[i][
                1] = self.Lloyd[i].move()

        if self.params["flag_plot"] == 1:
            self._update_plot()

        self.step += 1

    def _setup_plot_with_barriers(self):
        """
        Efficient plot setup with barrier visualization
        """
        plt.ion()  # Enable interactive mode
        self.fig, self.ax1 = plt.subplots(figsize=(10, 8))

        # Set equal aspect ratio and labels
        self.ax1.set_aspect('equal', adjustable='box')
        self.ax1.set_xlabel("X Position (m)", fontsize=12)
        self.ax1.set_ylabel("Y Position (m)", fontsize=12)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_title("BlueROV Simulation with Lloyd Algorithm",
                           fontsize=14)

        # Set plot limits based on basin dimensions
        self.ax1.set_xlim(self.params["xlim"][0] - 0.1,
                          self.params["xlim"][1] + 0.1)
        self.ax1.set_ylim(self.params["ylim"][0] - 0.1,
                          self.params["ylim"][1] + 0.1)

        # Plot barriers (walls and obstacles)
        self._plot_barriers()

        # Initialize robot and goal position plots for efficient updates
        self._initialize_robot_plots()

        plt.tight_layout()

    def _plot_barriers(self):
        """
        Plot basin walls and obstacles
        """
        # Get barrier coordinates
        barriers = def_barriers(self.params["dx"])

        # Separate basin walls from obstacles based on position
        basin_points = []
        obstacle_points = []

        for point in barriers:
            x, y = point
            # Points on the basin boundary are basin walls
            if (abs(x - self.params["xlim"][0]) < 1e-6
                    or abs(x - self.params["xlim"][1]) < 1e-6
                    or abs(y - self.params["ylim"][0]) < 1e-6
                    or abs(y - self.params["ylim"][1]) < 1e-6):
                basin_points.append(point)
            else:
                obstacle_points.append(point)

        # Plot basin walls
        if basin_points:
            basin_array = np.array(basin_points)
            self.ax1.scatter(basin_array[:, 0],
                             basin_array[:, 1],
                             c='black',
                             s=2,
                             alpha=0.8,
                             label='basin Walls')

        # Plot obstacles
        if obstacle_points:
            obstacle_array = np.array(obstacle_points)
            self.ax1.scatter(obstacle_array[:, 0],
                             obstacle_array[:, 1],
                             c='red',
                             s=3,
                             alpha=0.9,
                             label='Obstacles')

        # Add basin boundary rectangle for clearer visualization
        basin_rect = patches.Rectangle(
            (self.params["xlim"][0], self.params["ylim"][0]),
            self.params["xlim"][1] - self.params["xlim"][0],
            self.params["ylim"][1] - self.params["ylim"][0],
            linewidth=3,
            edgecolor='black',
            facecolor='none',
            label='basin Boundary')
        self.ax1.add_patch(basin_rect)

        # Add obstacle rectangle for better visualization
        _, obstacle_limits = get_limits()
        width = abs(np.round(obstacle_limits[0][0] - obstacle_limits[0][1], 3))
        height = abs(np.round(obstacle_limits[1][0] - obstacle_limits[1][1], 3))
        obstacle_rect = patches.Rectangle((obstacle_limits[0][0], obstacle_limits[1][0]),
                                          width,
                                          height,
                                          linewidth=2,
                                          edgecolor='red',
                                          facecolor='red',
                                          alpha=0.7) # alpha ist Deckkraft
        self.ax1.add_patch(obstacle_rect)

        self.barriers_plotted = True

    def _initialize_robot_plots(self):
        """
        Initialize plot objects for robots, goals, and trajectories for efficient updates
        """
        colors = ['blue', 'green', 'orange', 'purple', 'brown', 'pink']

        for i in range(self.params["N"]):
            color = colors[i % len(colors)]

            # Robot position (circle with robot size)
            robot_circle = plt.Circle(self.current_positions[i],
                                      self.params["size"][i],
                                      color=color,
                                      alpha=0.7,
                                      label=f'Robot {i+1}' if i < 3 else "")
            self.ax1.add_patch(robot_circle)
            self.robot_plots.append(robot_circle)

            # Goal position (cross marker)
            goal_plot, = self.ax1.plot(self.goal_positions[i][0],
                                       self.goal_positions[i][1],
                                       'x',
                                       color=color,
                                       markersize=10,
                                       markeredgewidth=3,
                                       label=f'Goal {i+1}' if i < 3 else "")
            self.goal_plots.append(goal_plot)

            # Trajectory line
            traj_plot, = self.ax1.plot([], [],
                                       '--',
                                       color=color,
                                       alpha=0.5,
                                       linewidth=1)
            self.trajectory_plots.append(traj_plot)

        # Add legend (only for first 3 robots to avoid clutter)
        self.ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

    def _update_plot(self):
        """
        Efficiently update robot positions and trajectories
        """
        if not hasattr(self, 'fig') or self.fig is None:
            return

        for i in range(self.params["N"]):
            # Update robot position
            self.robot_plots[i].center = self.current_positions[i]

            # Update trajectory
            self.trajectories[i].append(self.current_positions[i].copy())
            if len(self.trajectories[i]) > 1:
                traj_array = np.array(self.trajectories[i])
                self.trajectory_plots[i].set_data(traj_array[:, 0],
                                                  traj_array[:, 1])

        # Update display
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Optional: Save frame for animation (uncomment if needed)
        # if self.step % 10 == 0:  # Save every 10th frame
        #     plt.savefig(f'frame_{self.step:04d}.png', dpi=100, bbox_inches='tight')

    def cleanup_plot(self):
        """
        Clean up matplotlib resources
        """
        if hasattr(self, 'fig') and self.fig is not None:
            plt.close(self.fig)

    def get_simulation_stats(self):
        """
        Get current simulation statistics
        """
        return {
            'step': self.step,
            'max_velocity': self.max_velocity,
            'robots_at_goal': int(sum(self.flag)),
            'convergence_flag': self.flag_convergence
        }
