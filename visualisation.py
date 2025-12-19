# -*- coding: utf-8 -*-
"""
Created on Fri Dec 19 09:10:16 2025

@author: Jules
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

PATH_res = 'C:/Users/ronne/Documents/Recherche/CyclingStability/very_simple_torque_driven_ocp_cycling.pkl'

opti_res = pd.read_pickle(PATH_res)

q_sol = opti_res['q_sol']
qdot_sol = opti_res['qdot_sol']
tau_sol = opti_res['tau_sol']
time = np.linspace(0, 1, len(q_sol[0]))

config = {'$x$'              : ('q_sol', 3, [0,0], '$[m]$'),
          '$y$'              : ('q_sol', 4, [0,0], '$[m]$'),
          '$\delta$'         : ('q_sol', 2, [1,0], '$[rad]$'),
          '$\phi$'           : ('q_sol', 6, [1,0], '$[rad]$'),
          '$\psi$'           : ('q_sol', 5, [2,0], '$[rad]$'),
          '$\\theta$'         : ('q_sol', 0, [1,0], '$[rad]$'),
          '$\\alpha$'       : ('q_sol', 7, [3,0], '$[rad]$'),
          '$\\beta$'        : ('q_sol', 1, [3,0], '$[rad]$'),
          '$u$'              : ('qdot_sol', 3, [0,1], '$[m/s]$'),
          '$v$'              : ('qdot_sol', 4, [0,1], '$[m/s]$'),
          '$\dot{\delta}$'   : ('qdot_sol', 2, [1,1], '$[rad/s]$'),
          '$\dot{\phi}$'     : ('qdot_sol', 6, [1,1], '$[rad/s]$'),
          '$\dot{\psi}$'     : ('qdot_sol', 5, [2,1], '$[rad/s]$'),
          '$\dot{\\theta}$'   : ('qdot_sol', 0, [1,1], '$[rad/s]$'),
          '$\dot{\\alpha}$' : ('qdot_sol', 7, [3,1], '$[rad/s]$'),
          '$\dot{\\beta}$'  : ('qdot_sol', 1, [3,1], '$[rad/s]$'),}
          # '$\tau_{sol}$'     : ('tau_sol',  0, [3,1], '[Nm]')}

fig, axs = plt.subplots(4, 3, figsize=(8,9))

for key, value in config.items():
    i, j = value[2]
    axs[i, j].plot(time, opti_res[value[0]][value[1]], label=f'{key}')
    if j == 0:
        axs[i, 2].plot(time, np.gradient(time,opti_res[value[0]][value[1]]), label=f'{key} dot', ls='--')
        axs[i, 2].legend()
        axs[i, 2].set_xlabel('Time [s]')
        
    axs[i, j].set_xlabel('Time [s]')
    axs[i, j].set_ylabel(f'{value[3]}')
    axs[i, j].legend()
plt.suptitle(f'{PATH_res.split('/')[-1]}')
plt.tight_layout()


