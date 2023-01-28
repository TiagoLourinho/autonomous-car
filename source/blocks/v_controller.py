import numpy as np
import scipy.optimize
import scipy.signal

import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "source"))

from constants import *


def get_energy_budget(d: float, v_avg: float, P0: float, m: float, M: float) -> float:
    """
    Calculates energy budget estimate
    """
    return m * (M * v_avg * d + P0 * d / v_avg)


def get_path_stretches(path: list) -> list:
    """
    Calculates the length of each stretch defined by the path in between 2 consecutive points
    of the computed path
    """
    i = 0
    stretches = []
    for i in range(len(path)):
        if i == 0:
            continue
        stretch_length = np.linalg.norm(np.array(path[i]) - np.array(path[i - 1]))
        stretches.append(stretch_length)
    return stretches


def stretch_angle(point0: list, point1: list, point2: list) -> float:
    """
    Computes the angle between 2 consecutive stretches (defined by consecutive
    three points in space)
    """
    vstretch1 = point1 - point0
    vstretch2 = point2 - point1
    # Compute length of each stretch
    lstretch1 = np.linalg.norm(vstretch1)
    lstretch2 = np.linalg.norm(vstretch2)
    # Compute angle between 2 consecutive stretches
    aux = np.dot(vstretch1, vstretch2) / (lstretch1 * lstretch2)
    if aux > 1.00:
        aux = 1
    beta_i = np.arccos(aux)

    return beta_i


def get_max_velocities(path: list, vmax: float) -> list:
    """
    Computes maximum velocity allowed for each stretch, based on if the path between 2 consecutive strethes
    is linear or a curve. If its a curve, the tighter it is the lower the maximum allowed velocity is
    """
    flag = 0
    len_deceleration = 4
    deccel_constant = 1.4
    velocities = []
    betas = []
    i = 0
    while i < len(path) - 1:
        i += 1
        if i <= 1:
            continue

        if flag:
            flag = 0
            for k in range(len_deceleration):
                if k == 0:
                    velocities.append(velocities[-1] / deccel_constant)
                else:
                    velocities.append(velocities[-1])

            i += len_deceleration - 1
            continue

        beta = stretch_angle(path[i - 2], path[i - 1], path[i])
        if beta < np.pi / 6:
            beta = 0
        betas.append(beta)

        if beta != 0:
            multiplier = 1 / beta * np.pi / 6 * 0.8

        if beta != 0 and len(velocities) == 0:
            velocities.append(vmax * multiplier)
        elif beta != 0 and beta < np.pi / 4 and i != len(path) - 1:
            velocities.append(vmax * multiplier * deccel_constant)
            flag = 1

        elif beta != 0 and beta < 2 * np.pi / 3 and i != len(path) - 1:
            velocities.append(vmax * multiplier * 0.8 * deccel_constant)
            flag = 1

        elif beta != 0 and i != len(path) - 1:  # security
            velocities.append(vmax * multiplier)
            flag = 1
        else:
            velocities.append(vmax)
    # extra one
    velocities.append(vmax)

    return velocities


class VelocityController:
    """
    Calculates reference velocities according to a energy budget and computes control signals to apply to the car
    """

    def __init__(self, path: list, Kw: float, Kv: float):
        self.path = path
        self.Kw = Kw
        self.Kv = Kv
        self.Kpos = 1.5

        self.M = M
        self.P0 = P0
        self.L = L
        self.en_multiplier = multiplier
        self.avg_vel = avg_velocity
        self.vmax = max_velocity
        self.full_vel_error = 0

        self.stretches = get_path_stretches(path)
        self.energy_budget = get_energy_budget(
            sum(self.stretches), self.avg_vel, self.P0, self.en_multiplier, self.M
        )
        self.max_velocities = get_max_velocities(path, self.vmax)
        self.min_velocities = np.ones_like(self.max_velocities) * 1e-6
        self.ref_vels = self.optimize_velocities(
            self.path,
            self.stretches,
            self.energy_budget,
            self.max_velocities,
            self.min_velocities,
            self.P0,
            self.M,
        )
        print(f"Computed REF velocities: {self.ref_vels}")

    def get_full_path_vel_error(self):
        return self.full_vel_error

    def following_trajectory(
        self, point: list, state: list, energy_used: float, curr_idx: int
    ) -> list:
        # Retrieve car state and velocities
        theta = state[2]
        phi = state[3]
        x_dot = state[4]
        y_dot = state[5]
        vel = np.sqrt(x_dot**2 + y_dot**2)

        # Calculate position error in car and world frame
        pos_error = point[0:2] - state[0:2]

        car_frame_rotation = np.array(
            [[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]]
        )
        pos_error_car_frame = car_frame_rotation @ pos_error

        # If car very close to target point don't change steering (lasy point)
        if np.linalg.norm(pos_error_car_frame) < 1e-6:
            u_ws = 0
        else:
            ws_error = np.arctan2(pos_error_car_frame[1], pos_error_car_frame[0]) - phi
            u_ws = self.Kw * ws_error
            self.last_error = ws_error

        # Retrieve current reference velocity
        #curr_idx = np.where(self.path == point[0:2])[0][0] - 1
        ref_vel = self.ref_vels[curr_idx]
        # Compute velocity error
        vel_error = ref_vel - vel
        self.full_vel_error += vel_error

        if curr_idx != len(self.stretches) - 1:
            u_v = self.Kv * vel_error if energy_used < self.energy_budget else 0
        else:
            # position controller for parking the car
            x_error_car_frame = pos_error_car_frame[0]
            ref_vel = self.Kpos * x_error_car_frame
            vel_error = ref_vel - vel
            u_v = self.Kv * vel_error if energy_used < self.energy_budget else 0
            # If we're close enough to destination stop sending inputs
            if vel < 0.01:
                return np.array([0, 0])

        if energy_used > self.energy_budget:  # the car ran out of energy
            vel_error = vel if vel > 0 else 0
            force_apply = -self.Kv * vel_error
            print("Car ran out of fuel.")
            quit()

        # print(f"BUDGET: {self.energy_budget}")
        # print(f"Energy used: {energy_used}")
        # print(f"VEL: {vel}")
        # print(f"REF_VEL: {ref_vel}")

        return np.array([u_v * np.cos(phi), u_ws])

    @staticmethod
    def optimize_velocities(
        path, stretches, energy_budget, max_velocities, min_velocities, P0, M
    ) -> list:
        """
        Computes energy budget and reference velocities to minimize the time taken to travel the path according
        to the energy budget
        """
        max_iter = 6000
        # scaler for stability purposes
        stabilizer = 600
        # N = len(stretches)

        def E_used(stretches_vel):
            return (
                M * np.dot(stretches_vel, stretches)
                + P0 * (np.divide(stretches, stretches_vel)).sum()
            )

        def cost_func(stretches_vel):  # Optimization cost, total time of travel
            path_travel_times = np.divide(stretches, stretches_vel)
            return path_travel_times.sum()

        def jacobian(stretches_vel):  # jacobian of travel time
            return -np.divide(stretches, stretches_vel**2)

        constraints = {
            "type": "ineq",
            "fun": lambda stretches_vel: energy_budget - E_used(stretches_vel),
        }
        # velocity boundaries
        bounds = list(zip(min_velocities, max_velocities))

        # initial guess at optimal velocities
        v_1 = np.ones_like(min_velocities)
        optimal_vels = scipy.optimize.minimize(
            cost_func,
            v_1,
            method="SLSQP",
            jac=jacobian,
            bounds=bounds,
            constraints=constraints,
            options={"maxiter": max_iter},
        )

        if not optimal_vels.success:  # print some info if optimization failed
            print(
                f"No possible solution for the defined budget: {energy_budget:.2f}.\n The energy used will be : {E_used(optimal_vels.x):.2f}"
            )
            quit()
        print(
            f"POSSIBLE solution for the defined budget: {energy_budget:.2f}.\n The approximated energy used is : {E_used(optimal_vels.x):.2f}"
        )

        # append final velocity for final path
        velocities = np.block([optimal_vels.x, 0])
        return velocities


def main():
    # cont = VelocityController(path)
    pass


if __name__ == "__main__":
    main()
