from dash import html, dcc
import plotly.graph_objs as go
import numpy as np
from plot_utils import mode_plot, multivar_plot
from utils import make_input_field
from .trajectory_generators import (
    TrajectoryGenerator, 
    ConstantTrajectoryGenerator,
    StepTrajectoryGenerator, 
    SinusoidalTrajectoryGenerator
)


class BaseController:
    """A base class to define the basic interface for controllers"""

    title = None

    controller_inputs = {}

    state_info = []  # list with dict containing name, value, description

    trajectory_generators: dict[str, type[TrajectoryGenerator]] = {
        "Constant": ConstantTrajectoryGenerator,
        "Step": StepTrajectoryGenerator,
        "Sinusoidal": SinusoidalTrajectoryGenerator,
    }  # dict of trajectory generator name to class

    @classmethod
    def make_state_initialization_field(cls, controller_inputs: dict, input_info: list, output_info: list, state_info: list) -> html.Div:
        """Generate the layout for a state initialization field.
        Should be overridden in child classes if the controller has internal states to be initialized."""
        return html.Div([])

    @classmethod
    def make_layout(cls, controller_inputs: dict, input_info: list, output_info: list, state_info: list) -> html.Div:
        """Generate the layout for the controller's input fields based on the cls.controller_inputs variable."""

        # Add trajectory generator dropdown to controller inputs
        controller_fields = [
            make_input_field(name, props, "controller", controller_inputs)
            for name, props in cls.controller_inputs.items()
        ]
        
        # Add trajectory generator selection dropdown
        trajectory_generator_field = make_input_field("trajectory_generator",
            {
                "type": "dropdown",
                "value": "Constant",
                "description": "Trajectory generator type",
                "options": list(cls.trajectory_generators.keys()),
            },
            "controller",
            controller_inputs
        )

        return html.Div(
            [
                html.H2(f"{cls.title} Inputs"),
                *controller_fields,
                cls.make_state_initialization_field(controller_inputs, input_info, output_info, state_info),
                html.H3("Trajectory Generation"),
                trajectory_generator_field,
                html.Div(id={"type": "trajectory-inputs", "controller": cls.title})
            ]
        )

    def __init__(self, trajectory_generator: str = "Constant", trajectory_generator_inputs: dict = {}, initial_state: dict = {}):
        """
        Base initialization method inititializes time and u
        TODO: is u really needed to be stored here
        """
        self.trajectory_generator = self.trajectory_generators[trajectory_generator]
        self.trajectory_generator_inputs = trajectory_generator_inputs

        self.t = np.ndarray([])  # Placeholder for time array
        self.state = np.ndarray([])  # Placeholder for internal state
        self.u = np.ndarray([])  # Placeholder for control input
        self.reference_trajectory = np.ndarray([])  # Placeholder for reference trajectory

    def initialize(
        self,
        A: np.ndarray,
        B: np.ndarray,
        C: np.ndarray,
        dt: float,
        t: np.ndarray,
        state_info: list[dict[str, str]],
        output_info: list[dict[str, str]],
    ):
        """
        Initialize the controller and set a time step.
        This method should be called before the first step.
        initializes dt, t, and u
        Validates matrix dimensions and initializes controller parameters.

        Args:
            A: the state space A matrix
            B: the state space B matrix
            C: the state space C matrix
            dt: the controller timestep
            t: the time array
            state_info: list of dictionaries containing state information
        """
        self.output_info = output_info

        # Validate matrix dimensions
        # TODO do really want to keep this here i feel like the validation maybe should be elsewhere
        state_dim = A.shape[0]
        if A.shape[1] != state_dim:
            raise ValueError(f"A matrix must be square, got shape {A.shape}")
        if B.shape[0] != state_dim:
            raise ValueError(
                f"B matrix rows must match state dimension, got shape {B.shape}"
            )
        if C.shape[1] != state_dim:
            raise ValueError(
                f"C matrix columns must match state dimension, got shape {C.shape}"
            )

        input_dim = B.shape[1]
        output_dim = C.shape[0]

        self.dt = dt
        self.t = t
        # Initialize control input array with proper dimensions
        self.u = np.zeros((len(self.t), input_dim))

        # initialize the reference trajectory
        self.reference_trajectory = self.trajectory_generator.generate(
            A, B, C, t, self.trajectory_generator_inputs
        )

    def step(self, y: np.ndarray, index: int) -> np.ndarray:
        """
        Calculate the control output and step the internal state of the system

        Args:
            y: The current output of the system.
            index: The current index in the time array.

        Returns:
            The control action (u) to be applied to the system.
        """
        return np.array([0.0])

    def make_state_plots(self) -> list[go.Figure]:
        """
        Plot all internal state variables of the controller on a single figure.

        Returns:
        The plot of the controller states
        """
        return [
            multivar_plot(
                self.state,
                self.t,
                [state["name"] for state in self.state_info],
                "Controller states over time",
            ),
            multivar_plot(
                self.reference_trajectory,
                self.t,
                [f"ref_{output}" for output in self.output_info],
                "Reference trajectory over time",
            )
        ]

    def make_analysis_plots(self, A: np.ndarray, B: np.ndarray, C: np.ndarray) -> list:
        """
        Define all of the analysis plots associated with the controller

        Args:
        A: the state space A matrix
        B the state space B matrix
        C: the state space C matrix


        Returns:
        a list of figures to include
        """
        return [self.mode_plot(A, B, C)]

    def mode_plot(self, A: np.ndarray, B: np.ndarray, C: np.ndarray) -> dcc.Graph:
        """
        Plots the modes of the system on the complex plane.

        Args:
        A: the state space A matrix
        B the state space B matrix
        C: the state space C matrix

        Returns:
        the figure containing the plot
        """
        eigenvalues = self.calculate_controlled_eigenvalues(A, B, C)
        return dcc.Graph(figure=mode_plot(eigenvalues, "Controlled System Modes"))

    def calculate_controlled_eigenvalues(
        self, A: np.ndarray, B: np.ndarray, C: np.ndarray
    ) -> np.ndarray:
        """
        Calculate the eigenvalues of the controlled system.

        Args:
        A: linearized A matrix
        B: linearized B matrix
        C: linearized C matrix

        Returns:
        an ndarray with the complex modes of the controlled system
        """
        raise NotImplementedError("This method should be implemented in subclasses")
