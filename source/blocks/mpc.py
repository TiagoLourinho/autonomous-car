import numpy as np
# Import do_mpc package:
# pip install do-mpc
import do_mpc
import math
from scipy import signal
from casadi import *

class MPC_Controller:
    """MPC controller for the steering wheel"""

    def __init__(self):
        model_type = 'continuous' # either 'discrete' or 'continuous'
        model = do_mpc.model.Model(model_type)

        x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
        y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
        theta = model.set_variable(var_type='_x', var_name='theta', shape=(1,1))
        phi = model.set_variable(var_type='_x', var_name='phi', shape=(1,1))
        # Two states for the desired (set) motor position:
        V = model.set_variable(var_type='_u', var_name='V')
        ws = model.set_variable(var_type='_u', var_name='ws')
        L=2.46

        state_next = vertcat(
        np.cos(theta)*np.cos(phi)*V,
        np.sin(theta)*np.cos(phi)*V,
        np.sin(phi)/L,
        ws,
        )

        model.set_rhs('x', state_next[0])
        model.set_rhs('y', state_next[1])
        model.set_rhs('theta', state_next[2])
        model.set_rhs('phi', state_next[3])

        model.setup()

        self.model = model
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.V= V
        self.ws = ws

    def define_objective(self, point: np.array, state0: np.array):
        mpc = do_mpc.controller.MPC(self.model)

        setup_mpc = {
            'n_horizon': 10,
            't_step': 0.1,
            'n_robust': 1,
            'store_full_solution': False,
        }
        mpc.set_param(**setup_mpc)

        mterm = 10*(self.x-point[0])**2 + 10*(self.y-point[1])**2 
        lterm = (self.x-point[0])**2 + (self.y-point[1])**2 

        mpc.set_objective(mterm=mterm, lterm=lterm)

        mpc.set_rterm(
        V=1e-2,
        ws=0
        )

        # Lower bounds on states:
        mpc.bounds['lower','_x', 'x'] = -10000
        mpc.bounds['lower','_x', 'y'] = -10000
        mpc.bounds['lower','_x', 'theta'] = -4*np.pi
        mpc.bounds['lower','_x', 'phi'] = -np.pi/3
        # Upper bounds on states
        mpc.bounds['upper','_x', 'x'] = 10000
        mpc.bounds['upper','_x', 'y'] = 10000
        mpc.bounds['upper','_x', 'theta'] = 4*np.pi
        mpc.bounds['upper','_x', 'phi'] = np.pi/3

        # Lower bounds on inputs:
        mpc.bounds['lower','_u', 'V'] = 0
        mpc.bounds['lower','_u', 'ws'] = 0
        # Lower bounds on inputs:
        mpc.bounds['upper','_u', 'V'] = 45
        mpc.bounds['upper','_u', 'ws'] = 45

        #mpc.scaling['_x', 'x'] = 2
        #mpc.scaling['_x', 'y'] = 2
        #mpc.scaling['_x', 'theta'] = 2
        #mpc.scaling['_x', 'phi'] = 2

        mpc.setup()
        return mpc
    
    def following_trajectory(self, point: np.array, state0: np.array):
        mpc = self.define_objective(point, state0)
        #state0 = np.array([state0[0], state0[1], state0[2], 0])

        simulator = do_mpc.simulator.Simulator(self.model)

        # Instead of supplying a dict with the splat operator (**), as with the optimizer.set_param(),
        # we can also use keywords (and call the method multiple times, if necessary):
        simulator.set_param(t_step = 0.1)

        simulator.setup()

        simulator.x0 = state0
        mpc.x0 = state0

        mpc.set_initial_guess()

        #Running the simulator
        u0 = mpc.make_step(state0)
        state0 = simulator.make_step(u0)

        return np.array([u0[0][0], u0[1][0]]), state0


def simulations(mpc, simulator):
    # Customizing Matplotlib:
    mpl.rcParams['font.size'] = 18
    mpl.rcParams['lines.linewidth'] = 3
    mpl.rcParams['axes.grid'] = True

    mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
    sim_graphics = do_mpc.graphics.Graphics(simulator.data)

    # We just want to create the plot and not show it right now. This "inline magic" supresses the output.
    fig, ax = plt.subplots(4, sharex=True, figsize=(16,9))
    fig.align_ylabels()

    for g in [sim_graphics, mpc_graphics]:
        # Plot the angle positions (x, y, theta, phi) on the first axis:
        g.add_line(var_type='_x', var_name='x', axis=ax[0])
        g.add_line(var_type='_x', var_name='y', axis=ax[0])
        g.add_line(var_type='_x', var_name='theta', axis=ax[1])
        g.add_line(var_type='_x', var_name='phi', axis=ax[1])

        # Plot the set motor positions (V, ws) on the second axis:
        g.add_line(var_type='_u', var_name='V', axis=ax[2])
        g.add_line(var_type='_u', var_name='ws', axis=ax[3])

    ax[0].set_ylabel('pos [m]')
    ax[1].set_ylabel('angle [rad]')
    ax[2].set_ylabel('v [m/s]')
    ax[3].set_ylabel('steer v [rad/s]')
    ax[3].set_xlabel('time [s]')

    #sim_graphics.plot_predictions(t_ind=0)
    sim_graphics.plot_results()
    # Reset the limits on all axes in graphic to show the data.
    sim_graphics.reset_axes()
    # Show the figure:
    fig
    fig.savefig('test.png')

def main():
    mpc = MPC_Controller()

    with open("../../trajectory.txt") as f:
        lines = f.readlines()
        xs = []
        ys = []
        thetas = []
        for line in lines:
            xs.append(float(line.split()[0]))
            ys.append(float(line.split()[1]))
            thetas.append(float(line.split()[2]))

    j=0
    point = np.array([0,0,0,0])
    for i in range(len(lines)):
        print("REF:"*30, [xs[i], ys[i]])
        while True:
            u0, point = mpc.following_trajectory(np.array([xs[i], ys[i]]), point)
            print("REF:"*30, [xs[i], ys[i]])
            print("POINT:", [point[0], point[1]])
            print(j)
            print(np.sqrt((xs[i]-point[0])**2 + (ys[i]-point[1])**2)**2)
            print(np.linalg.norm(np.array([xs[i], ys[i]]) - np.array([point[0], point[1]]))**2)
            if np.sqrt((xs[i]-point[0])**2 + (ys[i]-point[1])**2)**2 < 2:
                j+=1
                if j==4:
                    quit()
                break

    sim = 0
    import matplotlib.pyplot as plt
    import matplotlib as mpl
    if sim:
        simulations(mpc, simulator)

if __name__ == "__main__":
    main()