import math
import numpy as np
import matplotlib.pyplot as plt
import copy
from BlueRovsInit import BlueRovsInit
from lloyd_path import Lloyd, aggregate


class bluerov_simulation:

    def __init__(self, parameters):
        self.params = parameters
        # self.tempo = 0 # tracks max velocity for reporting
        if self.params["N"] > 6:
            print("Maximale Anzahl an Bluerovs ist 6 aufgrund der Tankgröße.")
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
        # self.file_path = 'test' + str(self.params["h"]) + '.txt' # File path to save the data
        self.current_positions_x = np.zeros(
            self.params["N"])  # initialization of current x positions
        self.current_positions_y = np.zeros(
            self.params["N"])  # initialization of current y positions
        self.currents_theta = np.zeros(
            self.params["N"])  # initialization of current orientations
        self.flag_convergence = 0  # Initialization of the flag that indicates if all the robots have entered their goal regions
        self.current_positions = None
        self.goal_positions = None
        self.Lloyd = None
        self.Lloyd2 = None  # without neighbors (virtual)
        self.BlueRovs = None  # BlueROV objects # will be initialized later
        self.beta = self.params["betaD"].copy(
        )  # Initialization of the spreading factor rho

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
                  np.delete(self.params["size"], i,
                            axis=0), self.params["k"][i], self.params["dx"],
                  self.params["dt"], self.params["encumbrance_barriers"])
            for i in range(self.params["N"])
        ]  # ggf noch v_max TODO

        # Initialisierung des virtuellen Lloyd-Algorithmus ohne Nachbarn
        self.Lloyd2 = [
            Lloyd(self.current_positions[i], self.params["radius"],
                  self.params["size"][i],
                  np.delete(self.params["size"], i,
                            axis=0), self.params["k"][i], self.params["dx"],
                  self.params["dt"], self.params["encumbrance_barriers"])
            for i in range(self.params["N"])
        ]  # ggf noch v_max TODO

    def simulate_step(self):
        for i in range(self.params["N"]):
            neighbour_positions = np.delete(self.current_positions[i],
                                            i,
                                            axis=0)

            # Update both Lloyd instances with Nachbarinformationen
            # Ergebnis sind zwei arrays mit den Nachbarpositionen bzw. der Größe der Nachbarn
            self.Lloyd[i].aggregate(neighbour_positions, self.beta[i],
                                    self.goal_positions[i])
            self.Lloyd2[i].aggregate([], self.beta[i], self.goal_positions[i])

            # Berechnung der neuen Zielpositionen (c1 und c2)
            # c1 mit Nachbarn 
            # c2 ohne Nachbarn
            self.c1[i], self.c2[i] = self.Lloyd[i].compute_centroid()
            self.c1_no_rotation[i], self.c2_no_rotation[i] = self.Lloyd2[i].compute_centroid()
            


