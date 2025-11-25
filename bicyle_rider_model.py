# -*- coding: utf-8 -*-
"""
Created on Tue Nov 25 11:08:14 2025

@author: Jules
"""

# =============================================================================
# The Default Bicycle Model
# From https://mechmotum.github.io/symbrim/tutorials/my_first_bicycle.html
# Let's start with a very basic non-linear bicycle without rider
# =============================================================================


# Requirements
# pip install symbrim
# pip install bicycleparameters
# pip install symmeplot


import warnings

import sympy as sm
import sympy.physics.mechanics as me

import symbrim as sb
from symbrim.bicycle import RigidRearFrameMoore, WhippleBicycleMoore
from symbrim.brim import SideLeanSeatSpringDamper
from symbrim.rider import PinElbowTorque, SphericalShoulderTorque


bicycle = sb.WhippleBicycle("bike_v1_0")
assert type(bicycle) is WhippleBicycleMoore
bicycle.rear_frame = sb.RigidRearFrame.from_convention("moore", "rear_frame")
assert type(bicycle.rear_frame) is RigidRearFrameMoore
bicycle.rear_wheel = sb.KnifeEdgeWheel("rear_wheel")
bicycle.rear_tire = sb.NonHolonomicTire("rear_tire")

### BEGIN SOLUTION
bicycle.ground = sb.FlatGround("ground")
bicycle.front_frame = sb.RigidFrontFrame("front_frame")
bicycle.front_wheel = sb.KnifeEdgeWheel("front_wheel")
bicycle.front_tire = sb.NonHolonomicTire("front_tire")
### END SOLUTION

assert len(bicycle.submodels) == 5
assert len(bicycle.connections) == 2

bicycle.define_all()
system = bicycle.to_system()

normal = bicycle.ground.get_normal(bicycle.ground.origin)
g = sm.symbols("g")
# disturbance = me.dynamicsymbols("disturbance")
steer_torque = me.dynamicsymbols("steer_torque")
system.apply_uniform_gravity(-g * normal)
# system.add_loads(
#     me.Force(bicycle.rear_frame.saddle.point, disturbance * bicycle.rear_frame.wheel_hub.axis)
# )
system.add_actuators(
    me.TorqueActuator(steer_torque, bicycle.rear_frame.steer_hub.axis,
                      bicycle.rear_frame.steer_hub.frame, bicycle.front_frame.steer_hub.frame)
)


# Before forming the EoMs we need to specify which generalized coordinates 
# and speeds are independent and which are dependent.

try:
    system.validate_system()
except ValueError as e:
    display(e)
system.q_ind = [*bicycle.q[:4], *bicycle.q[5:]]
system.q_dep = [bicycle.q[4]]
system.u_ind = [bicycle.u[3], *bicycle.u[5:7]]
system.u_dep = [*bicycle.u[:3], bicycle.u[4], bicycle.u[7]]
system.validate_system()

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    eoms = system.form_eoms(constraint_solver="CRAMER")
    
    
#The equations of motions are generated as a 
#                                   "sympy.matrices.dense.MutableDenseMatrix"
    

#%% Parametrization

from pathlib import Path

import bicycleparameters as bp
import numpy as np


bike_params = bp.Bicycle("Browser", pathToData='data')
# bike_params.add_rider("Jason", reCalc=True)
    
constants = bicycle.get_param_values(bike_params)
constants[g] = 9.81  # Don't forget to specify the gravitational constant.

print('Constants of the model:')
constants

missing_symbols = bicycle.get_all_symbols().difference(constants.keys())

print('Is there any missing constant? -->')
missing_symbols

#%% Simulation

import numpy as np
from simulator import Simulator

simu = Simulator(system)

simu.constants = constants
simu.initial_conditions = {
    **{xi: 0.0 for xi in system.q.col_join(system.u)},
    bicycle.q[4]: 0.314,  # Initial guess rear frame pitch.
    bicycle.u[5]: -3.0 / constants[bicycle.rear_wheel.radius],  # Rear wheel angular velocity.
}
roll_rate_idx = len(system.q) + system.u[:].index(bicycle.u[3])
max_roll_rate, max_torque = 0.2, 10
simu.inputs = {
    # disturbance: lambda t, x: (30 + 30 * t) * np.sin(t * 2 * np.pi),
    steer_torque: lambda t, x: -max_torque * max(-1, min(x[roll_rate_idx] / max_roll_rate, 1)),
}

simu.initialize()
print('Initial Conditions:')
simu.initial_conditions


simu.solve([0,5], solver="solve_ivp")

#%% Visualization

import matplotlib.pyplot as plt
from IPython.display import HTML
from matplotlib.animation import FuncAnimation
from scipy.interpolate import CubicSpline

from symbrim.utilities.plotting import Plotter

# Create some functions to interpolate the results.
x_eval = CubicSpline(simu.t, simu.x.T)
r_eval = CubicSpline(simu.t, [[cf(t, x) for cf in simu.inputs.values()]
                                 for t, x in zip(simu.t, simu.x.T)])
p, p_vals = zip(*simu.constants.items())
# max_disturbance = r_eval(simu.t)[:, tuple(simu.inputs.keys()).index(disturbance)].max()

# Plot the initial configuration of the model
fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(8, 8))
plotter = Plotter.from_model(bicycle, ax=ax)
plotter.add_vector(disturbance * bicycle.rear_frame.wheel_hub.axis / max_disturbance,
                   bicycle.rear_frame.saddle.point, name="disturbance", color="r")
plotter.lambdify_system((system.q[:] + system.u[:], simu.inputs.keys(), p))
plotter.evaluate_system(x_eval(0.0), r_eval(0.0), p_vals)
plotter.plot()
X, Y = np.meshgrid(np.arange(-1, 10, 0.5), np.arange(-1, 3, 0.5))
ax.plot_wireframe(X, Y, np.zeros_like(X), color="k", alpha=0.3, rstride=1, cstride=1)
ax.invert_zaxis()
ax.invert_yaxis()
ax.set_xlim(X.min(), X.max())
ax.set_ylim(Y.min(), Y.max())
ax.view_init(19, 14)
ax.set_aspect("equal")
ax.axis("off")

fps = 30
ani = plotter.animate(
    lambda ti: (x_eval(ti), r_eval(ti), p_vals),
    frames=np.arange(0, simu.t[-1], 1 / fps),
    blit=False)
display(HTML(ani.to_jshtml(fps=fps)))






