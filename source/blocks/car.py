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

        self.__state = np.zeros(4) if initial_position is None else initial_position
        self.__L = L

    def get_length(self) -> float:
        return self.__L

    def get_current_state(self) -> np.array:
        return self.__state

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
                [np.cos(self.__state[2]) * np.cos(self.__state[3]), 0], 
                [np.sin(self.__state[2]) * np.cos(self.__state[3]), 0], 
                [np.sin(self.__state[3]) / self.__L               , 0], 
                [0                                                , 1]
            ]
        )
        # fmt: on

        self.__state = self.__state + time_step * np.matmul(model, np.array([v, omega]))
