from dash import html, dcc
import plotly.graph_objs as go
import numpy as np
from plot_utils import mode_plot, multivar_plot


class BaseController:
    """A base class to define the basic interface for controllers"""

    title = None

    controller_inputs = {}

    state_info = []  # list with dict containing name, value, description

    @classmethod
    def make_layout(cls):
        """Generate the layout for the controller's input fields based on the cls.controller_inputs variable."""
        return html.Div(
            [
                html.H2(f"{cls.title} Inputs"),
                *[
                    html.Div(
                        [
                            html.Label(props["description"]),
                            dcc.Input(
                                id={
                                    "type": "controller-input",
                                    "name": name,
                                },
                                type=props["type"],
                                value=props["value"],
                                debounce=True,
                            ),
                        ]
                    )
                    for name, props in cls.controller_inputs.items()
                ],
            ]
        )

    def __init__(self):
        """
        Base initialization method inititializes time and u
        TODO: is u really needed to be stored here
        """
        self.t = np.ndarray([])  # Placeholder for time array
        self.state = np.ndarray([])  # Placeholder for internal state
        self.u = np.ndarray([])  # Placeholder for control input

    def initialize(
        self,
        A: np.ndarray,
        B: np.ndarray,
        C: np.ndarray,
        dt: float,
        t: np.ndarray,
        state_info: list[dict[str, str]],
    ):
        """
        Initialize the controller and set a time step.
        This method should be called before the first step.
        initializes dt, t, and u

        Args:
            A: the state space A matrix
            B: the state space B matrix
            C: the state space C matrix
            dt: the controller timestep
            t: the time array
        """
        self.dt = dt
        self.t = t
        self.u = np.zeros(
            len(self.t)
        )  # Initialize control input array TODO is this needed

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
