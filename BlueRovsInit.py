import numpy as np

class BlueRovsInit:
    def __init__(self, parameters):
        start_positions = []
        goal_positions = []
        for i in range(parameters["N"]):
            if i < 3:
                x_start = 0.3 + i * 0.7
                x_goal = x_start
                y_start = 3.7
                y_goal = 0.3
            else:
                x_start = 0.3 + (i - 3) * 0.7
                x_goal = x_start
                y_start = 3.2
                y_goal = 0.8
            start_positions.append((x_start, y_start))
            goal_positions.append((x_goal, y_goal))

        self.start_positions = np.array(start_positions)
        np.random.shuffle(goal_positions)
        self.goal_positions = np.array(goal_positions)
