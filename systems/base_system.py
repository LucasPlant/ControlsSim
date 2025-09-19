from dash import html, dcc
import numpy as np
import plotly.graph_objs as go
from controllers import BaseController
from plot_utils import mode_plot, multivar_plot


class BaseSystem:
    """A base class to define the basic interface for systems."""

    # TODO add some documentation to what these fields are for

    title = None

    system_inputs = {}

    simulation_inputs = {}

    state_info = []  # list with dict containing name, value, description

    output_info = []  # list of strings with names

    input_info = []  # list of strings with names

    # Dictionary mapping controller names to controller classes that are compatible with this system
    allowed_controllers: dict[str, type[BaseController]] = {}

    @classmethod
    def make_layout(
        cls, system_inputs: dict | None = None, controller_inputs: dict | None = None
    ) -> html.Div:
        """Generate the layout for the system's input fields based on the cls.system_inputs variable.

        Returns:
        A div containing the layout for the systems input fields
        """

        def make_input_fields(inputs: dict) -> list:
            """
            Makes the gui input fields for the system.

            Args:
            inputs: a dict containing the systems inputs

            Returns:
            A list of fields that will be displayed in a DIV
            """
            input_fields = []
            # Parameter inputs
            for name, props in inputs.items():
                input_fields.extend(
                    [
                        html.Label(props["description"]),
                        dcc.Input(
                            id={"type": "input", "source": "system", "name": name},
                            type=props["type"],
                            value=props["value"],
                            debounce=True,
                        ),
                    ]
                )
            return input_fields

        def make_state_inputs(state_info: list[dict[str, str]]) -> list:
            """
            Makes the inputs for the state initialization

            Args:
            state_info: dict containing the info to make the state fields

            Returns:
            items to put in the input field DIV
            """
            input_fields = []
            # State Initialization inputs
            for idx, props in enumerate(state_info):
                input_fields.extend(
                    [
                        html.Label(props["name"] + ": " + props["description"]),
                        dcc.Input(
                            id={
                                "type": "input",
                                "source": "system",
                                "name": f"state_{idx}",
                            },
                            type="number",
                            value=props["value"],
                            debounce=True,
                        ),
                    ]
                )

            return input_fields

        return html.Div(
            [
                html.H1(cls.title),
                # System Inputs
                html.H2("System Inputs"),
                html.Div(make_input_fields(cls.system_inputs)),
                html.H2("Simulation Inputs"),
                html.Div(make_input_fields(cls.simulation_inputs)),
                html.H2("State Initialization"),
                html.Div(make_state_inputs(cls.state_info)),
                # Controller Selection
                html.H2("Select Controller"),
                dcc.Dropdown(
                    id={"type": "input", "source": "system", "name": "controller_type"},
                    options=[
                        {"label": k, "value": k} for k in cls.allowed_controllers.keys()
                    ],
                    value=(
                        list(cls.allowed_controllers.keys())[0]
                        if cls.allowed_controllers
                        else None
                    ),
                    clearable=False,
                ),
                # Container for controller-specific inputs - this will be populated by callback
                html.Div(id={"type": "controller-inputs", "system": cls.__name__}),
            ]
        )

    def __init__(
        self,
        dt: float,
        final_time: float,
        controller_type: str,
        controller_inputs: dict,
    ):
        """
        Initialize the system with a controller and simulation parameters.

        Args:
            dt: The time step for the simulation.
            final_time: The total time for the simulation.
            controller_type: The name of the controller to use (from allowed_controllers).
            controller_inputs: The inputs for the selected controller.
        """
        self.dt = dt
        self.final_time = final_time
        # Initial state of the system; should be overridden by subclasses
        self.initial_state = np.array([])

        # Initialize controller
        controller_class = self.allowed_controllers[controller_type]
        self.controller = controller_class(**controller_inputs)

    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        The state space model of the system.

        Args:
            x: The current state of the system.
            u: The control input to the system.
        Returns:
            The derivative of the state xdot.
        """
        raise NotImplementedError("This method should be implemented in subclasses")

    def g(self, x: np.ndarray) -> np.ndarray:
        """
        The output function of the system.

        Args:
            x: The current state of the system.
        Returns:
            The output of the system.
        """
        raise NotImplementedError("This method should be implemented in subclasses")

    def A(self) -> np.ndarray:
        """
        The linearized state matrix A of the system.
        """
        raise NotImplementedError("Subclasses must implement the A method.")

    def B(self) -> np.ndarray:
        """
        The linearized input matrix B of the system.
        """
        raise NotImplementedError("Subclasses must implement the B method.")

    def C(self) -> np.ndarray:
        """
        The linearized output matrix C of the system.
        """
        raise NotImplementedError("Subclasses must implement the C method.")

    def simulate(self) -> None:
        """
        Simulate the system using numerical integration techniques.
        Currently uses Forward Euler integration.
        """
        # Initialize the states of the system
        self.t = np.arange(0, self.final_time, self.dt)
        self.x = np.zeros((len(self.t), len(self.initial_state)))
        self.x[0, :] = self.initial_state

        # Get input/output dimensions from B and C matrices
        # TODO instead have the system creator make these for the potential for nonlinear control ect
        input_dim = self.B().shape[1]
        output_dim = self.C().shape[0]

        # Initialize u and y with proper dimensions for MIMO
        self.u = np.zeros((len(self.t), input_dim))
        self.y = np.zeros((len(self.t), output_dim))
        self.y[0, :] = self.g(self.x[0, :])

        self.controller.initialize(
            self.A(), self.B(), self.C(), self.dt, self.t, self.state_info
        )

        self.runge_kutta_4th_order()

    def forward_euler(self):
        """TODO"""
        # Forward Euler integration
        for i in range(0, len(self.t) - 1):
            self.u[i, :] = self.controller.step(self.y[i, :], i)

            # Update state
            self.x[i + 1, :] = (
                self.x[i, :] + self.f(self.x[i, :], self.u[i, :]) * self.dt
            )
            self.y[i + 1, :] = self.g(self.x[i + 1, :])

    def runge_kutta_4th_order(self):
        """TODO"""
        # Runge-Kutta 4th Order integration
        for i in range(0, len(self.t) - 1):
            # Calculate the control input for this time step
            # the u is not involved in the runge kutta steps as we are dealing with a digital controller
            # TODO Eventually we want the controller to support other update rates slower than the simulation rate
            self.u[i, :] = self.controller.step(self.y[i, :], i)

            k1 = self.f(self.x[i, :], self.u[i, :])
            k2 = self.f(self.x[i, :] + 0.5 * self.dt * k1, self.u[i, :])
            k3 = self.f(self.x[i, :] + 0.5 * self.dt * k2, self.u[i, :])
            k4 = self.f(self.x[i, :] + self.dt * k3, self.u[i, :])

            # Update state
            self.x[i + 1, :] = self.x[i, :] + (self.dt / 6) * (
                k1 + 2 * k2 + 2 * k3 + k4
            )
            self.y[i + 1, :] = self.g(self.x[i + 1, :])

    def make_simulation_plots(self):
        """
        Makes the plots to run after simulation
        Returns:
        A div containing plots made after the simulation
        """
        figures = [
            self.make_animation(),
            self.output_plot(),
            self.control_output_plot(),
            self.state_plot(),
            *self.controller.make_state_plots(),
        ]
        return html.Div([dcc.Graph(figure=fig) for fig in figures])

    def make_analysis_fields(self):
        """
        Makes plots to see based on current inputs

        Returns:
        A div containing plots run before the simulation
        """
        figures = [
            self.mode_plot(),
            *self.controller.make_analysis_plots(self.A(), self.B(), self.C()),
        ]
        return html.Div(figures)

    def make_animation(self) -> go.Figure:
        """
        Create an animation of the system
        """
        raise NotImplementedError("This method should be implemented in subclasses")

    def output_plot(self) -> go.Figure:
        """
        Plots the output of the system over time.

        Returns:
        A figure with the plot
        """
        return multivar_plot(self.y, self.t, self.output_info, "System Output (y)")

    def control_output_plot(self) -> go.Figure:
        """
        Plots the control output (u) over time.

        Returns:
        A figure with the plot
        """
        return multivar_plot(self.y, self.t, self.input_info, "Control input (u)")

    def state_plot(self) -> go.Figure:
        """
        Makes the plot of the system states over time

        Returns:
        A figure with the system state plots
        """
        return multivar_plot(
            self.x,
            self.t,
            [state["name"] for state in self.state_info],
            "System states over time",
        )

    def mode_plot(self) -> dcc.Graph:
        """
        Return a plot of the system's modes on the complex plane.

        Returns:
        A graph with the plot
        """
        eigenvalues = np.linalg.eigvals(self.A())
        return dcc.Graph(figure=mode_plot(eigenvalues, "System Modes"))
