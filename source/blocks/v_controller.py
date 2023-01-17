import numpy as np
import scipy.optimize
import scipy.signal


def get_energy_budget(d:float, v_avg: float, P0: float, m: float, M: float) -> float:
    """
    Calculates energy budget estimate
    """
    return m*(M*v_avg*d + P0*d/v_avg)

def get_path_stretches(path: list) -> list:
    """
    Calculates the length of each stretch defined by the path in between 2 consecutive points 
    of the computed path
    """
    i=0
    stretches = []
    for i in range(len(path)):
        if i==0:
            continue
        stretch_length = np.linalg.norm(np.array(path[i]) - np.array(path[i-1]))
        stretches.append(stretch_length)
    return stretches

def stretch_angle(point0: list, point1: list, point2: list) -> float:
    """
    Computes the angle between 2 consecutive stretches (defined by consecutive 
    three points in space)
    """
    vstretch1 = point1 - point0
    vstretch2 = point2 - point1
    #Compute length of each stretch
    lstretch1 = np.linalg.norm(vstretch1)
    lstretch2 = np.linalg.norm(vstretch2)
    #Compute angle between 2 consecutive stretches
    aux = np.dot(vstretch1, vstretch2) / (lstretch1 * lstretch2)
    if aux > 1.00:
        aux=1
    beta_i = np.arccos(aux)
    if (lstretch1 * lstretch2) ==0:
        print(point0)
        print(point1)
        print(point2)

    return beta_i

def get_max_velocities(path: list, vmax: float) -> list:
    """
    Computes maximum velocity allowed for each stretch, based on if the path between 2 consecutive strethes 
    is linear or a curve. If its a curve, the tighter it is the lower the maximum allowed velocity is
    """
    velocities = []
    for i in range(len(path)):
        if i <= 1:
            continue
        beta = stretch_angle(path[i-2], path[i-1], path[i])
        if beta < np.pi/6:
            beta=0
        velocities.append(vmax*(1-beta*1/np.pi))
    #extra one
    velocities.append(vmax*(1-beta*1/np.pi))

    return velocities

"""
def merge_velocities(max_velocities: list) -> list:
     _, idx = np.unique(max_velocities, axis=0, return_index=True)
     idxes = np.sort(idx)
     return idxes
"""

class VelocityController:
    """
    Calculates reference velocities according to a energy budget and computes control signals to apply to the car
    """
    def __init__(self, path: list):
        self.path = path

    def get_ref_velocities(self):
        """
        Computes reference velocities for the controller
        """
        velocities = self.optimize_velocities(self.path)
        return velocities


    @staticmethod
    def optimize_velocities(path) -> list:
        """
        Computes energy budget and reference velocities to minimize the time taken to travel the path according
        to the energy budget 
        """
        M = 1190
        P0 = 500
        stretches = get_path_stretches(path)
        energy_budget = get_energy_budget(sum(stretches), 30, 500, 1.3, M)
        max_velocities = get_max_velocities(path, 70)
        min_velocities = np.ones_like(max_velocities) * 1e-6
        idxes = merge_velocities(max_velocities)
        max_iter = 3000
        #scaler for stability purposes
        stabilizer = 256
        #N = len(stretches)
        print(len(stretches))
        print(len(max_velocities))

        def E_used(stretches_vel):
            return M*np.dot(stretches_vel,stretches) + P0*(np.divide(stretches, stretches_vel)).sum()

        def cost_func(stretches_vel):  # Optimization cost, total time of travel
            path_travel_times = np.divide(stretches, stretches_vel)
            return path_travel_times.sum()/stabilizer

        def jacobian(stretches_vel):  # jacobian of travel time
            return -np.divide(stretches, stretches_vel**2)/stabilizer

        

        constraints = ({'type': 'ineq', 'fun': lambda stretches_vel: E_used(stretches_vel) - energy_budget})
        # velocity boundaries
        bounds = list(zip(min_velocities, max_velocities))
        # initial guess at optimal velocities
        v_1 = np.ones_like(min_velocities)
        sol = scipy.optimize.minimize(cost_func, v_1, method='SLSQP', jac=jacobian, bounds=bounds,
                                      constraints=constraints, options={"maxiter": max_iter})

        if not sol.success: # print some info if optimization failed
            print(f"No possible solution for the defined budget: {energy_budget:.2f}.\n The energy used will be : {E_used(sol.x):.2f}")
        
        # append final velocity for final path
        velocities = np.block([sol.x, 0])
        return velocities



def main():
    #cont = VelocityController(path)
    pass
    

if __name__ == "__main__":
    main()