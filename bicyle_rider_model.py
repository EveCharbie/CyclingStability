# -*- coding: utf-8 -*-
"""
Created on Tue Nov 25 11:08:14 2025

@author: Jules + EveCharbie
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
# conda install -c conda-forge casadi


import warnings
from IPython.display import display
import numpy as np

import sympy as sm
import sympy.physics.mechanics as me

import symbrim as sb
from symbrim.bicycle import RigidRearFrameMoore, WhippleBicycleMoore
from symbrim.brim import SideLeanSeatSpringDamper
from symbrim.rider import PinElbowTorque, SphericalShoulderTorque
from sympy.utilities.lambdify import lambdify


bicycle = sb.WhippleBicycle("bike_v1_0")
assert type(bicycle) is WhippleBicycleMoore
bicycle.rear_frame = sb.RigidRearFrame.from_convention("moore", "rear_frame")
assert type(bicycle.rear_frame) is RigidRearFrameMoore
bicycle.rear_wheel = sb.KnifeEdgeWheel("rear_wheel")
bicycle.rear_tire = sb.NonHolonomicTire("rear_tire")


bicycle.ground = sb.FlatGround("ground")
bicycle.front_frame = sb.RigidFrontFrame("front_frame")
bicycle.front_wheel = sb.KnifeEdgeWheel("front_wheel")
bicycle.front_tire = sb.NonHolonomicTire("front_tire")


assert len(bicycle.submodels) == 5
assert len(bicycle.connections) == 2

bicycle.define_all()
system = bicycle.to_system()

normal = bicycle.ground.get_normal(bicycle.ground.origin)


# Add loads and actuators


# Gravity
g = sm.symbols("g")
system.apply_uniform_gravity(-g * normal)

# Disturbance
disturbance = me.dynamicsymbols("disturbance")
system.add_loads(
    me.Force(bicycle.rear_frame.saddle.point, 
             disturbance * bicycle.rear_frame.wheel_hub.axis))

# Steer torque
steer_torque = me.dynamicsymbols("steer_torque")
system.add_actuators(
    me.TorqueActuator(steer_torque, 
                      bicycle.rear_frame.steer_hub.axis,
                      bicycle.rear_frame.steer_hub.frame, 
                      bicycle.front_frame.steer_hub.frame))


# Before forming the EoMs we need to specify which generalized coordinates 
# and speeds are independent and which are dependent.


    
    
system.q_ind = [*bicycle.q[:4], *bicycle.q[5:]]
system.q_dep = [bicycle.q[4]]
system.u_ind = [bicycle.u[3], *bicycle.u[5:7]]
system.u_dep = [*bicycle.u[:3], bicycle.u[4], bicycle.u[7]]
system.validate_system()

try:
    system.validate_system()
except ValueError as e:
    print("\n\nERROR : ")
    display(e)

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

print('\n\nConstants of the model:')
print(constants)

missing_symbols = bicycle.get_all_symbols().difference(constants.keys())

print('\n\nIs there any missing constant? -->')
print(missing_symbols)

#%% Simulation

if False:
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
        disturbance: lambda t, x: (30 + 30 * t) * np.sin(t * 2 * np.pi),
        steer_torque: lambda t, x: -max_torque * max(-1, min(x[roll_rate_idx] / max_roll_rate, 1)),
    }

    simu.initialize()
    print('Initial Conditions:')
    simu.initial_conditions


    simu.solve([0,5], solver="solve_ivp")

#%% Visualization

if False:
    
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
    max_disturbance = r_eval(simu.t)[:, tuple(simu.inputs.keys()).index(disturbance)].max()
    
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


#%% Conversion


from sympy import lambdify, symbols
from sympy.physics.mechanics import *
import casadi as cas


def eval_num_full(
        system, #system.System returns this error :
                #'AttributeError: 'System' object has no attribute 'System'
        x,
        tau,
        distu,
):
        
    # M_m @ Xd + F_m = 0
    M_m = system.mass_matrix_full
    F_m = system.forcing_full
    
    _p, _p_vals = zip(*constants.items())
    _x = system.q.col_join(system.u)
    steer_torque = dynamicsymbols('steer_torque')
    disturbance = dynamicsymbols('disturbance')
    
    n = np.shape(M_m)[0]

    # This fails with "The requested array has an inhomogeneous shape after 2 dimensions. The detected shape was (16, 16) + inhomogeneous part."
    # f_M_m = lambdify(
    #     ( _x, _p, steer_torque, disturbance),
    #      M_m.reshape(n, n), cse=True)
    # M_m_num = f_M_m(x, _p_vals, tau, distu)
    
    f_F_m = lambdify(
        ( _x, _p, steer_torque, disturbance),
         F_m.reshape(1, n), cse=True)
    F_m_num = f_F_m(x, _p_vals, tau, distu)
        
    return F_m_num


def sympy_to_casadi_function(
        function_name: str,
        sympy_expr: sm.matrices.dense.MutableDenseMatrix,
        sympy_var: tuple[sm.matrices.immutable.ImmutableDenseMatrix],
        constants: dict[str, float],
):
    def safe_blockcat(matrix):
        """
        Make sure every element of a matrix is a CasADi expression, so that blockcat can work.
        """
        rows = []
        print(matrix)
        for row in matrix:
            new_row = []
            for elem in row:
                # print(elem)
                # replace raw ints/floats with CasADi constants
                if type(elem) == sm.core.numbers.NaN:
                    elem = cas.SX(0)  # TODO: see why there are NaNs !
                elif isinstance(elem, (int, float)):
                    elem = cas.SX(elem)
                new_row.append(elem)
            rows.append(new_row)
        return cas.blockcat(rows)


    variable_list = [
        cas.SX.sym("bike_v1_0_q1", 1, 1),
        cas.SX.sym("bike_v1_0_q2", 1, 1),
        cas.SX.sym("bike_v1_0_q3", 1, 1),
        cas.SX.sym("bike_v1_0_q4", 1, 1),
        cas.SX.sym("bike_v1_0_q6", 1, 1),
        cas.SX.sym("bike_v1_0_q7", 1, 1),
        cas.SX.sym("bike_v1_0_q8", 1, 1),
        cas.SX.sym("bike_v1_0_q5", 1, 1),
        cas.SX.sym("bike_v1_0_u4", 1, 1),
        cas.SX.sym("bike_v1_0_u6", 1, 1),
        cas.SX.sym("bike_v1_0_u7", 1, 1),
        cas.SX.sym("bike_v1_0_u1", 1, 1),
        cas.SX.sym("bike_v1_0_u2", 1, 1),
        cas.SX.sym("bike_v1_0_u3", 1, 1),
        cas.SX.sym("bike_v1_0_u5", 1, 1),
        cas.SX.sym("bike_v1_0_u8", 1, 1),
        cas.SX.sym("steer_torque", 1, 1),
        cas.SX.sym("disturbance", 1, 1),
    ]

    # x
    bike_v1_0_q1 = variable_list[0]
    bike_v1_0_q2 = variable_list[1]
    bike_v1_0_q3 = variable_list[2]
    bike_v1_0_q4 = variable_list[3]
    bike_v1_0_q6 = variable_list[4]
    bike_v1_0_q7 = variable_list[5]
    bike_v1_0_q8 = variable_list[6]
    bike_v1_0_q5 = variable_list[7]
    bike_v1_0_u4 = variable_list[8]
    bike_v1_0_u6 = variable_list[9]
    bike_v1_0_u7 = variable_list[10]
    bike_v1_0_u1 = variable_list[11]
    bike_v1_0_u2 = variable_list[12]
    bike_v1_0_u3 = variable_list[13]
    bike_v1_0_u5 = variable_list[14]
    bike_v1_0_u8 = variable_list[15]
    x_list = variable_list[:16]
    # steer torque
    steer_torque = variable_list[16]
    # disturbance
    disturbance = variable_list[17]


    casadi_mapping = {
       'ImmutableDenseMatrix': safe_blockcat,
       'MutableDenseMatrix': safe_blockcat,
       # 'ImmutableDenseMatrix': cas.blockcat,
       # 'MutableDenseMatrix': cas.blockcat,
       'Abs': cas.fabs,
       'sin': cas.sin,
       'cos': cas.cos,
       'tan': cas.tan,
       'asin': cas.asin,
       'acos': cas.acos,
       'atan': cas.atan,
       'sinh': cas.sinh,
       'cosh': cas.cosh,
       'tanh': cas.tanh,
       'asinh': cas.asinh,
       'acosh': cas.acosh,
       'atanh': cas.atanh,
       'exp': cas.exp,
       'log': cas.log,
       'sqrt': cas.sqrt,
      'bike_v1_0_q1(t)': bike_v1_0_q1,
      'bike_v1_0_q2(t)': bike_v1_0_q2,
      'bike_v1_0_q3(t)': bike_v1_0_q3,
      'bike_v1_0_q4(t)': bike_v1_0_q4,
      'bike_v1_0_q6(t)': bike_v1_0_q6,
      'bike_v1_0_q7(t)': bike_v1_0_q7,
      'bike_v1_0_q8(t)': bike_v1_0_q8,
      'bike_v1_0_q5(t)': bike_v1_0_q5,
      'bike_v1_0_u4(t)': bike_v1_0_u4,
      'bike_v1_0_u6(t)': bike_v1_0_u6,
      'bike_v1_0_u7(t)': bike_v1_0_u7,
      'bike_v1_0_u1(t)': bike_v1_0_u1,
      'bike_v1_0_u2(t)': bike_v1_0_u2,
      'bike_v1_0_u3(t)': bike_v1_0_u3,
      'bike_v1_0_u5(t)': bike_v1_0_u5,
      'bike_v1_0_u8(t)': bike_v1_0_u8,
      'steer_torque': steer_torque,
      'disturbance': disturbance,
       }
    # Add constants
    for name, value in constants.items():
        casadi_mapping[str(name)] = float(value)

    # sympy_expr = sympy_expr.simplify() #Can help to reduce the expr complexity
    f = lambdify(sympy_var, sympy_expr.simplify(), modules=[casadi_mapping])
    f(x_list, steer_torque, disturbance)
    casadi_func = cas.Function(function_name, variable_list, [f(x_list, steer_torque, disturbance)])
    # ERROR: casadi expressions not iterable by design

    return casadi_func


# Matrices extraction

# M_d @ Xd + F_d = 0
M_m = system.mass_matrix_full
F_m = system.forcing_full


from sympy_to_casadi_v2 import generate_model_file


variable_list = ['q1','q2','q3','q4','q5','q6','q7','q8',
                 'u1','u2','u3','u4','u5','u6','u7','u8',
                 'steer_torque', 'disturbance']



generate_model_file('model_d',['M_m','F_m'], [M_m, F_m], 
                    variable_list, constants)


from model_files.model_d import *


M_m_inv = cas.inv(M_m)
RHS = M_m_inv@F_m


f_RHS = cas.Function(
    'RHS', 
    list_variables + list_constants, 
    [RHS])

x = [1] * np.shape(list_variables)[0]
k = list(constants.values())

RHS_num = np.array(f_RHS(*x+k)).astype(float)

print(RHS_num)


# #%% Numerical Evaluation

# """
# The idea is to check if the conversion from sympy to casadi is correct.
# Evaluating the mass matric and the forcing vector with numerical values
# from the sympy and casadi expression then check if we get the same values.
# """

# _x = system.q.col_join(system.u)
# x = np.zeros(np.shape(_x))
# # tau = np.ones(1)
# # distu = np.ones(1)
# value_list = [0] * np.shape(_x)[0] + [1] + [1]

# print(eval_num_full(system, x, 1, 1))

# # Convertir M_m et F_m en CasADi
# f_M_func = sympy_to_casadi_function(
#     "M_d",
#     M_d,
#     (_x, steer_torque, disturbance),
#     constants,
# )
# f_F_func = sympy_to_casadi_function(
#     "F_d",
#     F_d,
#     (_x, steer_torque, disturbance),
#     constants,
# )

# # Erreur à ce niveau là
# print(np.array(f_M_func(*value_list)).astype(float))
# print(np.array(f_F_func(*value_list)).astype(float))



