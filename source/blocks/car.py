import numpy as np


class Car:
    """Car model"""

    def __init__(self, initial_position: np.array = None, L: float = 2.46):
        """
        Parameters
        ----------

        initial_position: np.array of shape (4, )
            Initial x, y, theta and phi

        L: float
            Distance between the front and back wheels (meters)
        """

        self.state = np.zeros(4) if initial_position is None else initial_position
        self.L = L

    def get_length(self) -> float:
        return self.L

    def get_current_state(self) -> np.array:
        return self.state

    def drive(self, v: float, omega: float, time_step: float):
        """Utilizes the model of the car to drive and returns the new position

        Parameters
        ----------

        v: float
            Linear velocity (meters per seconds)

        omega: float
            Steering angular velocity (radians per second)

        time_step: float
            Time step to use in Euler method (seconds)
        """

        # fmt: off
        model = np.array(
            [
                [np.cos(self.state[2]) * np.cos(self.state[3]), 0], 
                [np.sin(self.state[2]) * np.cos(self.state[3]), 0], 
                [np.sin(self.state[3]) / self.L               , 0], 
                [0                                                , 1]
            ]
        )
        # fmt: on

        self.state = self.state + time_step * np.matmul(model, np.array([v, omega]))
