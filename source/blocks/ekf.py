import numpy as np
from threading import Lock


class EKF:
    """Thread safe Extended Kalman Filter

    To simplify the implementation the EKF parameters are static (specific to the implementation of the problem)
    """

    def __init__(self, initial_estimate: np.array, predict_frequency: float):
        """
        For more info, see here: https://youtu.be/E-6paM_Iwfc?t=3622
        This one also helps (different connotation): https://www.kalmanfilter.net/multiSummary.html
        """

        self.car_L = 2.46
        self.time_step = 1 / predict_frequency

        self.state = np.zeros(shape=(8,))
        self.state[:2] = initial_estimate

        self.cov = np.identity(8)

        self.lock = Lock()

    def predict(self, control: np.array) -> None:
        """Predict step"""

        with self.lock:
            A = np.array(
                [
                    [1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0, 0, 0],
                    [0, 0, 0, 1, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                ]
            )

            B = np.array(
                [
                    [self.time_step * np.cos(self.state[2]) * np.cos(self.state[3]), 0],
                    [self.time_step * np.sin(self.state[2]) * np.cos(self.state[3]), 0],
                    [self.time_step * np.sin(self.state[3]) / self.car_L, 0],
                    [0, self.time_step],
                    [np.cos(self.state[2]) * np.cos(self.state[3]), 0],
                    [np.sin(self.state[2]) * np.cos(self.state[3]), 0],
                    [np.sin(self.state[3]) / self.car_L, 0],
                    [0, 1],
                ]
            )

            # fmt:off
            G = np.array(
                [
                    [1, 0, -control[0]*self.time_step*np.sin(self.state[2])*np.cos(self.state[3]), -control[0]*self.time_step*np.cos(self.state[2])*np.sin(self.state[3]), 0, 0, 0, 0],
                    [0, 1, control[0]*self.time_step*np.cos(self.state[2])*np.cos(self.state[3]), -control[0]*self.time_step*np.sin(self.state[2])*np.sin(self.state[3]), 0, 0, 0, 0],
                    [0, 0, 1, control[0]*self.time_step*np.cos(self.state[3])/self.car_L, 0, 0, 0, 0],
                    [0, 0, 0, 1, 0, 0, 0, 0],
                    [1, 0, -control[0]*np.sin(self.state[2])*np.cos(self.state[3]), -control[0]*np.cos(self.state[2])*np.sin(self.state[3]), 0, 0, 0, 0],
                    [0, 1, control[0]*np.cos(self.state[2])*np.cos(self.state[3]), -control[0]*np.sin(self.state[2])*np.sin(self.state[3]), 0, 0, 0, 0],
                    [0, 0, 1, control[0]*np.cos(self.state[3])/self.car_L, 0, 0, 0, 0],
                    [0, 0, 0, 1, 0, 0, 0, 0],
                ]
            )
            # fmt:on

            R = np.array(
                [
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                ]
            )

            g = lambda controls, state: A @ state + B @ controls

            self.state = g(control, self.state)
            self.cov = G @ self.cov @ G.T + R

    def update(self, measurements: np.array, sensor: str) -> None:
        """Update step"""
        with self.lock:

            if sensor.lower() == "gps":
                A = np.array(
                    [
                        [1, 0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0, 0, 0],
                    ]
                )

                H = np.array(
                    [
                        [1, 0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0, 0, 0],
                    ]
                )

                h = lambda state: A @ state

                Q = np.array(
                    [
                        [1, 0],
                        [0, 1],
                    ]
                )
            elif sensor.lower() == "imu":
                A = np.array(
                    [
                        [0, 0, 0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 1, 0],
                    ]
                )

                H = np.array(
                    [
                        [0, 0, 0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 1, 0],
                    ]
                )

                h = lambda state: A @ state

                Q = np.array(
                    [
                        [1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1],
                    ]
                )

            kalman_gain = self.cov @ H.T @ np.linalg.inv(H @ self.cov @ H.T + Q)
            self.state = self.state + kalman_gain @ (measurements - h(self.state))
            self.cov = (np.identity(8) - kalman_gain @ H) @ self.cov

    def get_current_state(self) -> np.array:
        """Returns the current state estimate"""
        with self.lock:

            return self.state
