import numpy as np


class EKF:
    """Extended Kalman Filter"""

    def init(
        self,
        initial_state: np.array,
        initial_covariance: np.array,
        G: np.array,
        H: np.array,
        Q: np.array,
        R: np.array,
        g: callable,
        h: callable,
    ):
        """
        For more info, see here: https://youtu.be/E-6paM_Iwfc?t=3622
        This one also helps (different connotation): https://www.kalmanfilter.net/multiSummary.html

        Parameters
        ----------
        initial_state: np.array
            Inital state estimate
        initial_covariance: np.array
            Initial covariance estimate
        G: np.array
            State Transition Matrix
        H: np.array
            Observation Matrix
        Q: np.array
            Process Noise Uncertainty
        R: np.array
            Measurement Uncertainty
        g: callable
            Expected inputs: previous state, present control
            Expected output: present state
        f: callable
            Expected inputs: present state
            Expected output: expected sensor readings
        """

        self.state = initial_state
        self.cov = initial_covariance
        self.G = G
        self.H = H
        self.Q = Q
        self.R = R
        self.g = g
        self.h = h

    def predict(self, control: np.array) -> None:
        """Predict step"""

        self.state = self.g(control, self.state)
        self.cov = self.G @ self.cov @ self.G.T + self.R

    def update(self, measurements: np.array) -> None:
        """Update step"""

        kalman_gain = (
            self.cov @ self.H.T @ np.linalg.inv(self.H @ self.cov @ self.H.T + self.Q)
        )

        self.state = self.state + kalman_gain * (measurements - self.h(self.state))
        self.cov = (np.identity(like=self.H) - kalman_gain * self.H) @ self.cov

    def get_current_estimate(self) -> tuple[np.array]:
        """Returns the current state estimate and estimate covariance"""

        return self.state, self.cov
