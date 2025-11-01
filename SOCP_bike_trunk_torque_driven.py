"""
States: q, qdot
Controls: cov (No tau control since only feedback control)
Algebraic states: M
Parameters: K, ref (upright position with only forward velocity)
Constants: motor noise, sensory noise
"""
import pickle
import casadi as cas
import numpy as np

from bioptim import (
    StochasticOptimalControlProgram,
    ObjectiveFcn,
    Solver,
    StochasticTorqueBiorbdModel,
    ObjectiveList,
    NonLinearProgram,
    BoundsList,
    InterpolationType,
    SocpType,
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
from bioptim.examples.utils import ExampleUtils

def sensory_reference(
    time: cas.MX | cas.SX,
    states: cas.MX | cas.SX,
    controls: cas.MX | cas.SX,
    parameters: cas.MX | cas.SX,
    algebraic_states: cas.MX | cas.SX,
    numerical_timeseries: cas.MX | cas.SX,
    nlp: NonLinearProgram,
):
    """
    This functions returns the sensory reference for the feedback gains.
    Vestibular: Head position + head velocity ... (what do we do with neck angle ?)
    Visual: 3D vector representing the orientation of the retina flow ?
    """
    # Mean states
    q = states[nlp.states["q"].index]
    qdot = states[nlp.states["qdot"].index]
    ref = cas.MX.zeros()  # TODO: implement
    return ref

def do_nothing():
    pass

def prepare_socp(
    biorbd_model_path: str,
    final_time: float,
    n_shooting: int,
    polynomial_degree: int,
    motor_noise_magnitude: cas.DM,
    sensory_noise_magnitude: cas.DM,
    q_opt: np.ndarray = None,
    qdot_opt: np.ndarray = None,
    tau_opt: np.ndarray = None,
    use_sx=False,
) -> StochasticOptimalControlProgram:
    """
    The initialization of an ocp
    Parameters
    ----------
    biorbd_model_path: str
        The path to the biorbd model
    final_time: float
        The time in second required to perform the task
    n_shooting: int
        The number of shooting points to define int the direct multiple shooting program
    polynomial_degree: int
        The degree of the polynomial used for the collocation integration
    motor_noise_magnitude: cas.DM
        The magnitude of the motor noise
    sensory_noise_magnitude: cas.DM
        The magnitude of the sensory noise
    use_sx: bool
        If SX should be used instead of MX

    Returns
    -------
    The OptimalControlProgram ready to be solved
    """

    auto_initialization = True if q_opt is not None else False
    initial_cov = cas.DM_eye(4) * np.array([1e-4, 1e-4, 1e-7, 1e-7])
    problem_type = SocpType.COLLOCATION(
        polynomial_degree=polynomial_degree,
        method="legendre",
        auto_initialization=auto_initialization,
        initial_cov=initial_cov,
    )

    bio_model = StochasticTorqueBiorbdModel(
        biorbd_model_path,  # TODO: change model
        problem_type=problem_type,
        with_cholesky=False,
        n_noised_states=4,
        n_noised_controls=2,
        sensory_noise_magnitude=sensory_noise_magnitude,
        motor_noise_magnitude=motor_noise_magnitude,
        sensory_reference=sensory_reference,
        n_references=4,  # This number must be in agreement with what is declared in sensory_reference
        n_feedbacks=4,
        use_sx=use_sx,
        friction_coefficients=np.array([[0.05, 0.025], [0.025, 0.05]]),  # Friction helps for stability
    )

    n_q = bio_model.nb_q
    n_qdot = bio_model.nb_qdot
    n_states = n_q * 2
    n_k = 2 * 4
    n_ref = 4
    n_m = 4 * 4
    n_cov = 4 * 4

    # Add objective functions
    objective_functions = ObjectiveList()
    objective_functions.add(
        ObjectiveFcn.Lagrange.MINIMIZE_CONTROL,
        node=Node.ALL_SHOOTING,
        key="tau",
        weight=1e3 / 2,
        quadratic=True
    )
    objective_functions.add(
        ObjectiveFcn.Lagrange.STOCHASTIC_MINIMIZE_EXPECTED_FEEDBACK_EFFORTS,
        node=Node.ALL_SHOOTING,
        weight=1e3 / 2,
        quadratic=True,
    )

    # Constraints
    constraints = ConstraintList()
    constraints.add(
        ConstraintFcn.TRACK_STATE,
        key="q",
        node=Node.START,
        target=np.array([0, 0]),
    )
    constraints.add(
        ConstraintFcn.TRACK_STATE,
        key="qdot",
        node=Node.START,
        target=np.array([0, 0]),
    )

    # Parameters
    parameters = ParameterList(use_sx=use_sx)
    parameters.add(
        "K",
        do_nothing,  # The function that modifies the biorbd model
        size=n_k,  # n_ref * n_q
        scaling=VariableScaling("K", np.ones((n_k, ))),
    )
    parameters.add(
        "ref",
        do_nothing,  # The function that modifies the biorbd model
        size=n_ref,
        scaling=VariableScaling("ref", np.ones((n_ref, ))),
    )

    parameter_bounds = BoundsList()
    parameter_bounds.add(
        "K",
        min_bound=np.ones((n_k, )) * -10,
        max_bound=np.ones((n_k, )) * 10,
        interpolation=InterpolationType.CONSTANT,
    )
    parameter_bounds.add(
        "ref",
        min_bound=np.ones((n_ref, )) * -1,
        max_bound=np.ones((n_ref, )) * 1,
        interpolation=InterpolationType.CONSTANT,
    )

    parameter_init = InitialGuessList()
    parameter_init["K"] = np.ones((n_k, )) * 0.01
    parameter_init["ref"] = np.zeros((n_ref, ))

    parameter_objectives = ParameterObjectiveList()
    parameter_objectives.add(
        ObjectiveFcn.Parameter.MINIMIZE_PARAMETER,
        key="K",
        weight=1,
        quadratic=True,
    )

    # Dynamics
    dynamics = DynamicsOptions(expand_dynamics=True) # Depends if the model dynamics is RAM expensive

    x_bounds = BoundsList()
    x_bounds.add("q", min_bound=[-10] * n_q, max_bound=[10] * n_q, interpolation=InterpolationType.CONSTANT)
    x_bounds.add(
        "qdot", min_bound=[-100] * n_qdot, max_bound=[100] * n_qdot, interpolation=InterpolationType.CONSTANT
    )

    # Initial guesses
    x_init = InitialGuessList()
    if q_opt is not None:
        x_init.add("q", initial_guess=q_opt, interpolation=InterpolationType.ALL_POINTS)
        x_init.add("qdot", initial_guess=qdot_opt, interpolation=InterpolationType.ALL_POINTS)
    else:
        x_init.add("q", initial_guess=[0] * n_q, interpolation=InterpolationType.CONSTANT)
        x_init.add("qdot", initial_guess=[0] * n_q, interpolation=InterpolationType.CONSTANT)

    u_init = InitialGuessList()
    a_init = InitialGuessList()
    if q_opt is None:
        a_init.add("m", initial_guess=[0.01] * n_m, interpolation=InterpolationType.CONSTANT)

        idx = 0
        cov_init_vector = np.zeros((n_states * n_states, 1))
        for i in range(n_states):
            for j in range(n_states):
                cov_init_vector[idx] = initial_cov[i, j]
        u_init.add("cov", initial_guess=cov_init_vector, interpolation=InterpolationType.CONSTANT)

    u_bounds = BoundsList()
    u_bounds.add(
        "cov", min_bound=[-cas.inf] * n_cov, max_bound=[cas.inf] * n_cov, interpolation=InterpolationType.CONSTANT
    )
    a_bounds = BoundsList()
    a_bounds.add("m", min_bound=[-cas.inf] * n_m, max_bound=[cas.inf] * n_m, interpolation=InterpolationType.CONSTANT)

    return StochasticOptimalControlProgram(
        bio_model,
        n_shooting,
        final_time,
        dynamics=dynamics,
        x_init=x_init,
        u_init=u_init,
        a_init=a_init,
        x_bounds=x_bounds,
        u_bounds=u_bounds,
        a_bounds=a_bounds,
        objective_functions=objective_functions,
        constraints=constraints,
        control_type=ControlType.CONSTANT_WITH_LAST_NODE,
        n_threads=1,
        problem_type=problem_type,
        use_sx=use_sx,
    )


def main():
    # --- Options --- #
    use_sx = True
    vizualize_sol_flag = True

    biorbd_model_path = ExampleUtils.folder + "/models/LeuvenArmModel.bioMod" # TODO: change model

    # --- Prepare the ocp --- #
    dt = 0.01
    final_time = 1.0  # TODO: see if we could go up to 10s
    n_shooting = int(final_time / dt)

    # --- Noise constants --- #
    # TODO: start small but go as high as plausible
    motor_noise_std = 0.05
    wPq_std = 3e-4
    wPqdot_std = 0.0024

    motor_noise_magnitude = cas.DM(np.array([motor_noise_std**2 / dt, motor_noise_std**2 / dt]))
    wPq_magnitude = cas.DM(np.array([wPq_std**2 / dt, wPq_std**2 / dt]))
    wPqdot_magnitude = cas.DM(np.array([wPqdot_std**2 / dt, wPqdot_std**2 / dt]))
    sensory_noise_magnitude = cas.vertcat(wPq_magnitude, wPqdot_magnitude)

    # Solver parameters
    solver = Solver.IPOPT(show_online_optim=False)
    solver.set_linear_solver("mumps")  # TODO: change for MA97
    solver.set_tol(1e-6)  # TODO: check which dynamics consistency is needed to answer the question
    solver.set_maximum_iterations(10000)
    # solver.set_hessian_approximation("limited-memory")
    solver.set_bound_frac(1e-8)
    solver.set_bound_push(1e-8)
    solver.set_nlp_scaling_method("none")  # TODO: see if this helps

    socp = prepare_socp(
        biorbd_model_path=biorbd_model_path,
        final_time=final_time,
        n_shooting=n_shooting,
        polynomial_degree=3,
        motor_noise_magnitude=motor_noise_magnitude,
        sensory_noise_magnitude=sensory_noise_magnitude,
        use_sx=use_sx,
    )

    sol_socp = socp.solve(solver)
    # sol_socp.graphs()

    states = sol_socp.stepwise_states(to_merge=SolutionMerge.NODES)
    controls = sol_socp.stepwise_controls(to_merge=SolutionMerge.NODES)
    algebraic_states = sol_socp.decision_algebraic_states(to_merge=SolutionMerge.NODES)
    parameters = sol_socp.decision_parameters()

    q_sol = states["q"]
    qdot_sol = states["qdot"]
    cov_sol = controls["cov"]
    m_sol = algebraic_states["m"]
    k_sol = parameters["k"]
    ref_sol = parameters["ref"]
    data = {
        "q_sol": q_sol,
        "qdot_sol": qdot_sol,
        "k_sol": k_sol,
        "ref_sol": ref_sol,
        "m_sol": m_sol,
        "cov_sol": cov_sol,
    }

    # --- Save the results --- #
    with open(f"very_simple_torque_driven_socp_cycling.pkl", "wb") as file:
        pickle.dump(data, file)

    # --- Visualize the results --- #
    # TODO: see if possible to add to pyorerun


if __name__ == "__main__":
    main()
