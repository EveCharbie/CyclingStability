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
)

from model_files.bioptim_model import BikeModel


def prepare_ocp(
    final_time: float,
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
        ConstraintFcn.TRACK_STATE,
        key="q",
        node=Node.START,
    )
    constraints.add(
        ConstraintFcn.TRACK_STATE,
        key="qdot",
        node=Node.START,
        min_bound=np.array([1, 0, 0, 0, 0, 0, 0, 0]),  # Start moving forward
        max_bound=np.array([100, 0, 0, 0, 0, 0, 0, 0]),  # Start moving forward
    )
    constraints.add(
        ConstraintFcn.TRACK_STATE,
        key="q",
        node=Node.END,
        min_bound=np.array([5, -10, 0, 0, 0, 0, 0, 0]),  # Start moving forward
        max_bound=np.array([5, 10, 0, 0, 0, 0, 0, 0]),  # Start moving forward
    )

    # Dynamics
    dynamics = DynamicsOptions(expand_dynamics=False)

    # Bounds
    x_bounds = BoundsList()

    q_min = np.zeros((8, 3))
    q_max = np.zeros((8, 3))
    q_min[:, 1] = [-10] * 8
    q_max[:, 1] = [10] * 8
    q_min[:, 2] = [5, -10, -10, -10, -10, -10, 0, -10]  # End further away with zero stead angle
    q_max[:, 2] = [5, 10, 10, 10, 10, 10, 0, 10]  # End further away with zero stead angle
    x_bounds.add(
        "q", min_bound=q_min, max_bound=q_max, interpolation=InterpolationType.CONSTANT_WITH_FIRST_AND_LAST_DIFFERENT
    )

    qdot_min = np.zeros((8, 3))
    qdot_max = np.zeros((8, 3))
    qdot_min[:, 0] = [5, 0, 0, 0, 0, 0, 0, 0]  # Start with some forward speed
    qdot_max[:, 0] = [5, 0, 0, 0, 0, 0, 0, 0]  # Start with some forward speed
    qdot_min[:, 1] = [-100] * 8
    qdot_max[:, 1] = [100] * 8
    qdot_min[:, 2] = [-100, -100, 0, 0, 0, -100, 0, -100]  # End with zero stear velocity and instability
    qdot_max[:, 2] = [100, 100, 0, 0, 0, 100, 0, 100]  # End with zero stear velocity and instability
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
    x_init.add("q", initial_guess=[0] * n_q, interpolation=InterpolationType.CONSTANT)
    x_init.add("qdot", initial_guess=[0] * n_q, interpolation=InterpolationType.CONSTANT)

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
    final_time = 1.0  # TODO: see if we could go up to 10s
    n_shooting = int(final_time / dt)

    # Solver parameters
    solver = Solver.IPOPT(show_online_optim=False)
    solver.set_linear_solver("mumps")  # TODO: change for MA97
    solver.set_tol(1e-6)  # TODO: check which dynamics consistency is needed to answer the question
    solver.set_maximum_iterations(10000)
    # solver.set_hessian_approximation("limited-memory")
    solver.set_bound_frac(1e-8)
    solver.set_bound_push(1e-8)
    solver.set_nlp_scaling_method("none")  # TODO: see if this helps

    ocp = prepare_ocp(
        final_time=final_time,
        n_shooting=n_shooting,
        polynomial_degree=3,
    )

    sol_ocp = ocp.solve(solver)
    # sol_socp.graphs()

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
    with open(f"very_simple_torque_driven_ocp_cycling.pkl", "wb") as file:
        pickle.dump(data, file)

    # --- Visualize the results --- #
    # TODO: see if possible to add to pyorerun


if __name__ == "__main__":
    main()
