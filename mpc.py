import numpy as np
# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
sys.path.append('../../')
# Import do_mpc package:
# pip install do-mpc
import do_mpc
import math


model_type = 'continuous' # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
theta = model.set_variable(var_type='_x', var_name='theta', shape=(1,1))
phi = model.set_variable(var_type='_x', var_name='phi', shape=(1,1))
# Two states for the desired (set) motor position:
V = model.set_variable(var_type='_u', var_name='V')
ws = model.set_variable(var_type='_u', var_name='ws')
L = 2.46

from casadi import *

state_next = vertcat(
    np.cos(theta)*np.cos(phi)*V,
    np.sin(theta)*np.cos(phi)*V,
    np.cos(phi)/L,
    ws,
)

model.set_rhs('x', state_next[0])
model.set_rhs('y', state_next[1])
model.set_rhs('theta', state_next[2])
model.set_rhs('phi', state_next[3])

model.setup()

mpc = do_mpc.controller.MPC(model)

setup_mpc = {
    'n_horizon': 5,
    't_step': 0.1,
    'n_robust': 1,
    'store_full_solution': True,
}
mpc.set_param(**setup_mpc)

mterm = x**2 + y**2 + phi**2 + theta**2
lterm = x**2 + y**2 + phi**2 + theta**2

mpc.set_objective(mterm=mterm, lterm=lterm)

mpc.set_rterm(
    V=1e-2,
    ws=1e-2
)

# Lower bounds on states:
mpc.bounds['lower','_x', 'x'] = -1000
mpc.bounds['lower','_x', 'y'] = -1000
mpc.bounds['lower','_x', 'theta'] = -4*np.pi
mpc.bounds['lower','_x', 'phi'] = -10
# Upper bounds on states
mpc.bounds['upper','_x', 'x'] = 1000
mpc.bounds['upper','_x', 'y'] = 1000
mpc.bounds['upper','_x', 'theta'] = 4*np.pi
mpc.bounds['upper','_x', 'phi'] = 10

# Lower bounds on inputs:
mpc.bounds['lower','_u', 'V'] = -10
mpc.bounds['lower','_u', 'ws'] = -10
# Lower bounds on inputs:
mpc.bounds['upper','_u', 'V'] = 10
mpc.bounds['upper','_u', 'ws'] = 10

#mpc.scaling['_x', 'x'] = 2
#mpc.scaling['_x', 'y'] = 2
#mpc.scaling['_x', 'theta'] = 2
#mpc.scaling['_x', 'phi'] = 2

mpc.setup()

simulator = do_mpc.simulator.Simulator(model)

# Instead of supplying a dict with the splat operator (**), as with the optimizer.set_param(),
# we can also use keywords (and call the method multiple times, if necessary):
simulator.set_param(t_step = 0.1)

simulator.setup()

#state0 = np.pi*np.array([1, 1, -1.5, 1, -1, 1, 0, 0]).reshape(-1,1)
#state0 = get_state()
state0= np.array([1, 1, 1, 1])
simulator.x0 = state0
mpc.x0 = state0

mpc.set_initial_guess()

#Simulations
import matplotlib.pyplot as plt
import matplotlib as mpl
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
ax[1].set_ylabel('angle position [rad]')
ax[2].set_ylabel('motor angle [rad]')
ax[3].set_ylabel('velocity [m/s]')
ax[3].set_xlabel('time [s]')

#Running the simulator
u0 = np.zeros((2,1))
for i in range(200):
    simulator.make_step(u0)


sim_graphics.plot_results()
# Reset the limits on all axes in graphic to show the data.
sim_graphics.reset_axes()
# Show the figure:
fig


"""
\begin{equation}
    \begin{bmatrix}
        \Dot{x}\\[6pt]
        \Dot{y}\\[6pt]
        \Dot{\theta}\\[6pt]
        \Dot{\phi}\\[6pt]
    \end{bmatrix}=
    \begin{bmatrix}
        \cos(\theta)\cos(\phi) & 0\\[6pt]
        \sin(\theta)\cos(\phi) & 0\\[6pt]
        \frac{\sin(\phi)}{2.46} & 0\\[6pt]
        0 & 1\\[6pt]
    \end{bmatrix}
    \begin{bmatrix}
        V\\[6pt]
        w_s\\[6pt]
    \end{bmatrix}
\end{equation}
"""