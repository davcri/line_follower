"""
TODO: implement a model of the robot car
"""

import math


class Simulation():
    def simulatedError(self, i):
        max_error = 12  # experimental value
        error = max_error*math.sin(i)

        return error
