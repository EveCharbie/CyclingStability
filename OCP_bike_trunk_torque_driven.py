"""
States: q, qdot
Controls: tau
"""

import pickle
import casadi as cas
import numpy as np

from bioptim import (
    OptimalControlProgram,
    ObjectiveFcn,
    Solver,
    ObjectiveList,
    NonLinearProgram,
    BoundsList,
    InterpolationType,
    Node,
    ConstraintList,
    ConstraintFcn,
    InitialGuessList,
    ControlType,
    ParameterList,
    ParameterObjectiveList,
    SolutionMerge,
    DynamicsOptions,
    VariableScaling,
    PenaltyController,
)

from model_files.bioptim_model import BikeModel


def custom_indep_dof_continuity(controller: PenaltyController) -> cas.MX:
    """
    Used to define continuity constraints on independent degrees of freedom only.
    The continuity of dependent degrees of freedom is ensured by holonomic constraints included in the dynamics of the system directly.
    """
    dependent_q_indices =    [0, 1, 2, 3,   5, 6, 7]
    dependent_qdot_indices = [         3,   5, 6,  ]
    indices_to_keep = (np.hstack((np.array(dependent_q_indices), np.array(dependent_qdot_indices) + 8))).tolist()

    if controller.control_type in (
            ControlType.CONSTANT,
            ControlType.CONSTANT_WITH_LAST_NODE,
    ):
        u = controller.controls.cx_start
    elif controller.control_type == ControlType.LINEAR_CONTINUOUS:
        u = cas.horzcat(controller.controls.cx_start, controller.controls.cx_end)
    else:
        raise NotImplementedError(f"Dynamics with {controller.control_type} is not implemented yet")

    t_span = controller.t_span.cx
    continuity = controller.states.cx_end[indices_to_keep]
    if controller.get_nlp.dynamics_type.ode_solver.is_direct_collocation:
        states_cx = cas.horzcat(*([controller.states.cx_start] + controller.states.cx_intermediates_list))
        algebraic_states_cx = cas.horzcat(
            *([controller.algebraic_states.cx_start] + controller.algebraic_states.cx_intermediates_list)
        )

        integrated = controller.integrate(
            t_span=t_span,
            x0=states_cx,
            u=u,
            p=controller.parameters.cx,
            a=algebraic_states_cx,
            d=controller.numerical_timeseries.cx,
        )
        continuity -= integrated["xf"][indices_to_keep]
        continuity = cas.vertcat(continuity, integrated["defects"][indices_to_keep])

    else:
        continuity -= controller.integrate(
            t_span=t_span,
            x0=controller.states.cx_start,
            u=u,
            p=controller.parameters.cx_start,
            a=controller.algebraic_states.cx_start,
            d=controller.numerical_timeseries.cx,
        )["xf"][indices_to_keep]

    return continuity


def prepare_ocp(
    final_time: float,
    final_position: float,
    n_shooting: int,
    polynomial_degree: int,
) -> OptimalControlProgram:

    bio_model = BikeModel()

    n_q = bio_model.nb_q

    # Add objective functions
    objective_functions = ObjectiveList()
    objective_functions.add(
        ObjectiveFcn.Lagrange.MINIMIZE_CONTROL, node=Node.ALL_SHOOTING, key="tau", weight=1e3 / 2, quadratic=True
    )

    # Constraints
    constraints = ConstraintList()
    constraints.add(
        custom_indep_dof_continuity,
        node=Node.ALL_SHOOTING,
        integrate=True,
        explicit_derivative=True,
        multi_thread=True,
    )

    # Dynamics
    dynamics = DynamicsOptions(expand_dynamics=False, skip_continuity=True)

    # Bounds
    x_bounds = BoundsList()

    q_min = np.zeros((8, 3))  # Start with everything at zero
    q_max = np.zeros((8, 3))  # Start with everything at zero
    q_min[:, 1] = [0,  # The bike should only translate forward
                   -5,
                   -10,
                   -10,
                   -10,
                   0,  # The rear wheel should only rotate forward
                   -10,
                   0 # The front wheel should only rotate forward
                   ]
    q_max[:, 1] = [5,
                   5,
                   10,
                   10,
                   10,
                   100, # The rear wheel should rotate forward
                   10,   # Zero stear angle at the end
                   100, # The front wheel should rotate forward
                   ]
    q_min[:, 2] = [final_position,  # The rear wheel contact point ends forward
                   0,  # The rear wheel contact point ends without lateral translation
                   -10,  # No somersault rotation (does this need to be constrained ?, Holonomic should take care of it ?)
                   -10,
                   -10,
                   0,  # The rear wheel should rotate forward
                   0,  # Zero stear angle at the end
                   0 # The front wheel should rotate forward
                   ]
    q_max[:, 2] = [final_position,   # The rear wheel contact point ends forward
                   0,   # The rear wheel contact point ends without lateral translation
                   10,   # No somersault rotation (does this need to be constrained ?, Holonomic should take care of it ?)
                   10,
                   10,
                   100, # The rear wheel should rotate forward
                   0,   # Zero stear angle at the end
                   100, # The front wheel should rotate forward
                   ]
    x_bounds.add(
        "q", min_bound=q_min, max_bound=q_max, interpolation=InterpolationType.CONSTANT_WITH_FIRST_AND_LAST_DIFFERENT
    )

    qdot_min = np.zeros((8, 3))
    qdot_max = np.zeros((8, 3))
    qdot_min[:, 0] = [
        1,  # Start with some forward translational speed
        0,  # Zero sideways translational speed
        0,
        0,
        0,
        1,  # Start with some positive forward rear wheel rotation speed
        0,
        1,  # Start with some positive forward front wheel rotation speed
    ]
    qdot_max[:, 0] = [
        10,  # Start with some forward translational speed
        0,  # Zero sideways translational speed
        0,
        0,
        0,
        100,  # Start with some positive forward rear wheel rotation speed
        0,
        100,  # Start with some positive forward front wheel rotation speed
    ]
    qdot_min[:, 1] = [-100] * 8
    qdot_max[:, 1] = [100] * 8
    qdot_min[:, 2] = [
        -100,
        -100,
        0,
        0,
        0,
        0,
        0,
        0,
    ]  # End with zero stear velocity and instability
    qdot_max[:, 2] = [
        100,
        100,
        0,
        0,
        0,
        100,
        0,
        100,
    ]  # End with zero stear velocity and instability
    x_bounds.add(
        "qdot",
        min_bound=qdot_min,
        max_bound=qdot_max,
        interpolation=InterpolationType.CONSTANT_WITH_FIRST_AND_LAST_DIFFERENT,
    )

    u_bounds = BoundsList()
    u_bounds.add("tau", min_bound=[-10], max_bound=[10], interpolation=InterpolationType.CONSTANT)

    # Initial guesses
    x_init = InitialGuessList()
    q_init = np.zeros((n_q, n_shooting + 1))
    q_init[0, :] = np.linspace(0, final_position, n_shooting + 1)  # Rear wheel x from 0 to final_position
    q_init[5, :] = np.linspace(0, final_position, n_shooting + 1)  # Distance / wheel circumference (assumed radius of 1m) * 2pi -> wheel angle
    q_init[7, :] = np.linspace(0, final_position, n_shooting + 1)  # Distance / wheel circumference (assumed radius of 1m) * 2pi -> wheel angle
    x_init.add("q", initial_guess=q_init, interpolation=InterpolationType.EACH_FRAME)
    qdot_init = [
        final_position/final_time,  # Start with some forward translational speed
        0,  # Zero sideways translational speed
        0,
        0,
        0,
        2*np.pi,  # Start with some positive forward rear wheel rotation speed
        0,
        2*np.pi,  # Start with some positive forward front wheel rotation speed
    ]
    x_init.add("qdot", initial_guess=qdot_init, interpolation=InterpolationType.CONSTANT)

    u_init = InitialGuessList()
    u_init.add("tau", initial_guess=[0], interpolation=InterpolationType.CONSTANT)

    return OptimalControlProgram(
        bio_model,
        n_shooting,
        final_time,
        dynamics=dynamics,
        x_init=x_init,
        u_init=u_init,
        x_bounds=x_bounds,
        u_bounds=u_bounds,
        objective_functions=objective_functions,
        constraints=constraints,
        control_type=ControlType.CONSTANT_WITH_LAST_NODE,
        n_threads=1,
        use_sx=False,
    )


def main():
    # --- Options --- #
    vizualize_sol_flag = True

    # --- Prepare the ocp --- #
    dt = 0.1
    final_time = 1  # TODO: see if we could go up to 10s
    final_position = 0.3  # 30 cm
    n_shooting = int(final_time / dt)

    # Solver parameters
    solver = Solver.IPOPT(show_online_optim=False, show_options=dict(show_bounds=True))
    solver.set_linear_solver("ma57")  # "mumps" is the simple but bad option, # TODO: change for MA97 ?
    solver.set_tol(1e-6)  # TODO: check which dynamics consistency is needed to answer the question
    solver.set_maximum_iterations(10000)
    # solver.set_hessian_approximation("limited-memory")
    solver.set_bound_frac(1e-8)
    solver.set_bound_push(1e-8)
    solver.set_nlp_scaling_method("none")  # TODO: see if this helps

    ocp = prepare_ocp(
        final_time=final_time,
        final_position=final_position,
        n_shooting=n_shooting,
        polynomial_degree=3,
    )

    sol_ocp = ocp.solve(solver)
    sol_ocp.graphs(show_bounds=True, save_name="results/very_simple_torque_driven_ocp_cycling_graph")

    states = sol_ocp.stepwise_states(to_merge=SolutionMerge.NODES)
    controls = sol_ocp.stepwise_controls(to_merge=SolutionMerge.NODES)

    q_sol = states["q"]
    qdot_sol = states["qdot"]
    tau_sol = controls["tau"]
    data = {
        "q_sol": q_sol,
        "qdot_sol": qdot_sol,
        "tau_sol": tau_sol,
    }

    # --- Save the results --- #
    with open(f"results/very_simple_torque_driven_ocp_cycling.pkl", "wb") as file:
        pickle.dump(data, file)

    # --- Visualize the results --- #
    # TODO: see if possible to add to pyorerun


if __name__ == "__main__":
    main()
