import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from lloyd_path import Lloyd
from bluerov_simulation import bluerov_simulation


def main():

    N = 5  # Anzahl der Bluerovs (max 6 aufgrund von Tankgröße)
    parameters = {
        "radius": 1,  # Half of the sensing radius: dimension of the cells r_{s,i}=r_{s}
        # Tank ist 2x4 Meter
        "xlim": (0, 2),  # Beckendimensionen in X-axis in m
        "ylim": (0, 4),  # Beckendimensionen in Y-axis in m
        "xlim_obstacle": (0.75, 1.25),  # Hindernisdimensionen in X-axis in m
        "ylim_obstacle": (1.95, 2.05),  # Hindernisdimensionen in Y-axis in m
        "N": N,  # Number of BlueROV2s
        "num_steps": 5000,  # Number of simulation steps
        "dx": 0.025,  # Space discretization [It introduce an approximation. The lower the better, but it is computationally expensive]
        "dt": 0.033,  # Time discretization 
        "d1": 0.4,  # d1 eq. (8) TODO scaling factor # orig: 0.1
        "d3": 0.8,  # d3 eq. (9) TODO scaling factor # orig: 0.1
        "beta_min": 0.1,  # Minimum value for spreading factor rho TODO scaling factor
        "betaD": [0.1] * N,  # Desired spreading factor \rho^D TODO scaling factor
        "size": [0.24] * N,  # BlueROV2s encumbrance (radius): \delta
        "k": [10] * N,  # Control parameter k_p TODO scaling factor
        "flag_plot": 1,
        "write_file": 1,
        "v_max": [5] * N,  # Maximum velocity for each robot TODO scaling factor # orig: 5
        "waiting_time": 400,  # waiting time after all the robots enter their goal regions.
        "h": 1,  # number to change each time you want to create a unique logging file name
        "encumbrance_barriers": 0.1  # control the distance between the robot and the barriers (inflate/deflate)
    }

    # create an instance of the bluerov_simulation class
    simulation = bluerov_simulation(parameters)

    # initialize the simulation
    simulation.initialize_simulation()

    # run the simulation
    try:
        for step in range(parameters["num_steps"]):
            simulation.simulate_step()

            # Optional: Print progress every 100 steps
            if step % 100 == 0:
                stats = simulation.get_simulation_stats()
                print(
                    f"Step {stats['step']}: {stats['robots_at_goal']}/{parameters['N']} robots at goal"
                )

    except KeyboardInterrupt:
        print("Simulation interrupted by user")
    finally:
        # Clean up plot resources
        simulation.cleanup_plot()
        print("Simulation completed")


if __name__ == "__main__":
    main()

# # Raumparameter
# raum_breite = 2.0  # Meter # nur in barriers.py definieren
# raum_laenge = 4.0   # Meter # nur in barriers.py definieren

# # Hindernis (zentral)
# hindernis_breite = 0.5  # Meter
# hindernis_laenge = 0.1   # Meter
# hindernis_x = (raum_breite - hindernis_breite) / 2
# hindernis_y = (raum_laenge - hindernis_laenge) / 2

# # Roboterparameter
# roboter_radius = 0.25  # Meter

# # Roboter (Start und Ziel) in euklidischen Koordinaten
# roboter1_start = [0.5, 0.5]
# roboter1_ziel = [1.5, 3.5]
# roboter2_start = [1.5, 0.5]
# roboter2_ziel = [0.5, 3.5]
# start = [roboter1_start, roboter2_start] # zugriff auf z.b. 1.5: start[1][0]
# ziel = [roboter1_ziel, roboter2_ziel]

# # # Beispielhafte Pfade (nur gerade Linien, ohne Kollisionsprüfung)
# # roboter1_pfad = np.linspace(roboter1_start, roboter1_ziel, 50)
# # roboter2_pfad = np.linspace(roboter2_start, roboter2_ziel, 50)

# # Pfade mit Lloyd-Algorithmus berechnen
# pfade = lloyd_path(start, ziel, None, roboter_radius)
# roboter1_pfad = pfade[0]
# roboter2_pfad = pfade[1]

# fig, ax = plt.subplots(figsize=(6, 12))

# # Raum zeichnen
# ax.set_xlim(0, raum_breite)
# ax.set_ylim(0, raum_laenge)
# ax.set_aspect('equal')
# ax.add_patch(patches.Rectangle((0, 0), raum_breite, raum_laenge, fill=False, edgecolor='black'))

# # Hindernis zeichnen
# ax.add_patch(patches.Rectangle((hindernis_x, hindernis_y), hindernis_breite, hindernis_laenge, color='gray'))

# # Roboterpfade zeichnen
# ax.plot(roboter1_pfad[:,0], roboter1_pfad[:,1], 'b--', label='Roboter 1 Pfad')
# ax.plot(roboter2_pfad[:,0], roboter2_pfad[:,1], 'r--', label='Roboter 2 Pfad')

# # Roboter als Kreise
# roboter1_circle = plt.Circle(roboter1_start, roboter_radius, color='blue', label='Roboter 1')
# roboter2_circle = plt.Circle(roboter2_start, roboter_radius, color='red', label='Roboter 2')
# ax.add_patch(roboter1_circle)
# ax.add_patch(roboter2_circle)

# ax.legend()
# plt.title('Simulation: Zwei Roboter im Raum mit Hindernis')
# plt.xlabel('x [m]')
# plt.ylabel('y [m]')

# def animate(i):
#     roboter1_circle.center = roboter1_pfad[i]
#     roboter2_circle.center = roboter2_pfad[i]
#     return roboter1_circle, roboter2_circle

# ani = FuncAnimation(fig, animate, frames=len(roboter1_pfad), interval=100, blit=True)
# plt.show()

# hindernis1 = np.array([[0.75, 1.95], [1.25, 1.95], [1.25, 2.05], [0.75, 2.05]])
# hindernis_x = (hindernis1[:, 0] / 0.01).astype(int)

# print(f'Hindernis x-Koordinaten in barriers-Indizes: {hindernis_x}')
