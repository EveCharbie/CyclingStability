"""
This script implements a custom model to work with bioptim.
"""
import pickle
import platform
import casadi as cas
from typing import Callable
from bioptim import (
    NonLinearProgram,
    DynamicsEvaluation,
    StateDynamics,
    DynamicsFunctions,
    ConfigureVariables,
    BiMapping,
)


class BikeModel(StateDynamics):

    def __init__(self):
        super().__init__()

        self.q = cas.SX.sym("q", 8)
        self.qdot = cas.SX.sym("qdot", 8)
        self.tau = cas.SX.sym("tau", 1)
        self.contact_types = ()
        self.fatigue = None
        self.list_variables: list[cas.SX.sym] = None  # Will be filled by declare_dynamics

        self.declare_constants()
        self.declare_dynamics()


    def declare_constants(self) -> None:

        if platform.system() == 'Windows':
            full_file_name = f'model_files\constants_d.pkl'
        else:
            full_file_name = f'model_files/constants_d.pkl'

        with open(full_file_name, "rb") as f:
            constants = pickle.load(f)

        self.constants = constants


    def declare_dynamics(self) -> None:

        from model_files.model_d import (
            list_variables,
            list_constants,
            M_m,
            F_m,
        )
        self.list_variables = list_variables

        M_m_inv = cas.inv(M_m)
        RHS = M_m_inv @ F_m

        f_RHS = cas.Function(
            'RHS',
            list_variables + list_constants,
            [RHS])

        k = list(self.constants.values())

        self.forward_dynamics = cas.Function(
            "forward_dynamics",
            [self.q, self.qdot, self.tau],
            [f_RHS(
                *([self.q[0],
                self.q[1],
                self.q[2],
                self.q[3],
                self.q[4],
                self.q[5],
                self.q[6],
                self.q[7],
                self.qdot[0],
                self.qdot[1],
                self.qdot[2],
                self.qdot[3],
                self.qdot[4],
                self.qdot[5],
                self.qdot[6],
                self.qdot[7],
                self.tau[0], 0] + k))]
        )

    def serialize(self) -> tuple[Callable, dict]:
        return BikeModel, dict(tata="tata")

    @property
    def nb_tau(self):
        return 2

    @property
    def nb_q(self):
        return 8

    @property
    def mass(self):
        return 1

    @property
    def name_dofs(self):
        return [f"dof_{i}" for i in range(self.nb_q)]


    @property
    def name(self) -> str:
        return "bike_model"

    @property
    def state_configuration_functions(self):
        return [
            lambda **kwargs: ConfigureVariables.configure_q(as_states=True, **kwargs),
            lambda **kwargs: ConfigureVariables.configure_qdot(as_states=True, **kwargs),
        ]

    @staticmethod
    def configure_tau(
        ocp,
        nlp,
        as_states: bool = False,
        as_controls: bool = True,
        as_algebraic_states: bool = False,
    ):
        name = "tau"
        name_tau = ["stear"]
        ConfigureVariables.configure_new_variable(
            name,
            name_tau,
            ocp,
            nlp,
            as_states=as_states,
            as_controls=as_controls,
            as_algebraic_states=as_algebraic_states,
            axes_idx=BiMapping([0], [0]),
        )

    @property
    def control_configuration_functions(self):
        return [self.configure_tau]

    @property
    def algebraic_configuration_functions(self):
        return []

    @property
    def extra_configuration_functions(self):
        return []

    def dynamics(
        self,
        time: cas.MX,
        states: cas.MX,
        controls: cas.MX,
        parameters: cas.MX,
        algebraic_states: cas.MX,
        numerical_timeseries: cas.MX,
        nlp: NonLinearProgram,
    ) -> DynamicsEvaluation:
        """
        Parameters
        ----------
        states: MX | SX
            The state of the system
        controls: MX | SX
            The controls of the system
        parameters: MX | SX
            The parameters acting on the system
        nlp: NonLinearProgram
            A reference to the phase
        Returns
        -------
        The derivative of the states in the tuple[MX | SX] format
        """
        q = DynamicsFunctions.get(nlp.states["q"], states)
        qdot = DynamicsFunctions.get(nlp.states["qdot"], states)
        tau = DynamicsFunctions.get(nlp.controls["tau"], controls)

        return DynamicsEvaluation(
            dxdt=cas.vertcat(qdot, self.forward_dynamics(q, qdot, tau)),
            defects=None,
        )
