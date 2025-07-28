from dash import html, dcc
import numpy as np
import plotly.graph_objs as go
from controllers import BaseController


class BaseSystem:
    """A base class to define the basic interface for systems."""

    # TODO add some documentation to what these fields are for

    title = None

    system_inputs = {}

    simulation_inputs = {}

    state_info = []  # list with dict containing name, value, description

    @classmethod
    def make_layout(cls):
        """Generate the layout for the system's input fields based on the cls.system_inputs variable."""

        def make_input_fields(inputs: dict) -> list:
            """Makes the gui input fields for the system."""
            input_fields = []
            # Parameter inputs
            for name, props in inputs.items():
                input_fields.extend(
                    [
                        html.Label(props["description"]),
                        dcc.Input(
                            id={"type": "system-input", "name": name},
                            type=props["type"],
                            value=props["value"],
                        ),
                    ]
                )
            return input_fields

        def make_state_inputs(state_info: list[dict[str, str]]) -> list:
            input_fields = []
            # State Initialization inputs
            for idx, props in enumerate(state_info):
                input_fields.extend(
                    [
                        html.Label(props["name"] + ": " + props["description"]),
                        dcc.Input(
                            id={"type": "system-input", "name": f"state_{idx}"},
                            type="number",
                            value=props["value"],
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
            ]
        )

    def __init__(self, controller: BaseController, dt: float, final_time: float):
        """
        Initialize the system with a controller and simulation parameters.

        Args:
            controller: An instance of a controller that implements the BaseController interface.
            dt: The time step for the simulation.
            final_time: The total time for the simulation.
        """
        self.controller = controller
        self.dt = dt
        self.final_time = final_time
        # Initial state of the system; should be overridden by subclasses
        self.state = np.array([])

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
        self.x = np.zeros((len(self.t), len(self.state)))
        self.x[0] = self.state
        self.u = np.zeros((len(self.t))) # TODO need to fix this for MIMO systems
        self.y = np.zeros(len(self.t))
        self.y[0] = self.g(self.x[0])

        self.controller.initialize(self.A(), self.B(), self.C(), self.dt, self.t)

        # Forward Euler integration
        # TODO other integration methods can be implemented later
        for i in range(0, len(self.t) - 1):
            self.u[i] = self.controller.step(self.y[i], i)

            # Update state
            self.x[i + 1] = self.x[i] + self.f(self.x[i], self.u[i]) * self.dt
            self.y[i + 1] = self.g(self.x[i + 1])

        self.state = self.x[-1]  # Update the state to the last computed state

    def make_simulation_plots(self):
        """
        Create a Div containing all plots for the mass-spring system.
        """
        figures = [
            self.make_animation(),
            self.output_plot(),
            self.control_output_plot(),
            *self.state_plots(),
            *self.controller.make_state_plots(),
        ]
        return html.Div([dcc.Graph(figure=fig) for fig in figures])

    def make_analysis_plots(self):
        """
        Create a Div containing all analysis plots for the mass-spring system.
        """
        figures = [
            self.mode_plot(),
            *self.controller.make_analysis_plots(self.A(), self.B(), self.C()),
        ]
        return html.Div([dcc.Graph(figure=fig) for fig in figures])

    def make_animation(self) -> go.Figure:
        """
        Create an animation of the mass-spring system
        """
        raise NotImplementedError("This method should be implemented in subclasses")

    def output_plot(self) -> go.Figure:
        """
        Plots the output of the system over time.
        TODO will need to be adjusted for MIMO systems when implemented.
        """
        # Determine axis limits based on max/min output
        y_min = np.min(self.y) - 0.5
        y_max = np.max(self.y) + 0.5

        # Output vs Time
        output_plot = go.Figure()
        output_plot.add_trace(go.Scatter(x=self.t, y=self.y, mode="lines"))
        output_plot.update_layout(
            title="System Output",
            xaxis_title="Time (s)",
            yaxis_title="Output (y)",
            yaxis=dict(range=[y_min, y_max]),
        )
        return output_plot
    
    def control_output_plot(self) -> go.Figure:
        """
        Plots the control output (u) over time.
        """
        # If u is 2D (e.g., shape (N, 1)), flatten for plotting
        control_output_plot = go.Figure()
        control_output_plot.add_trace(go.Scatter(x=self.t, y=self.u, mode="lines"))
        control_output_plot.update_layout(
            title="Control Output (u) Over Time",
            xaxis_title="Time (s)",
            yaxis_title="Control Output (u)",
        )
        return control_output_plot

    def state_plots(self) -> list[go.Figure]:
        """
        Return a list of state plots for the system.
        """
        state_plots = []
        for i in range(self.x.shape[1]):
            state_plot = go.Figure()
            state_plot.add_trace(
                go.Scatter(x=self.t, y=self.x[:, i], mode="lines", name=f"State {i+1}")
            )
            state_name = self.state_info[i]["name"]
            state_plot.update_layout(
                title=f"State {i+1}: {state_name} Over Time",  # TODO: eventually make a mapping to something meaningful
                xaxis_title="Time (s)",
                yaxis_title=f"State {i+1}",
            )
            state_plots.append(state_plot)
        return state_plots

    def mode_plot(self) -> go.Figure:
        """
        Return a plot of the system's modes on the complex plane.
        """
        eigenvalues = np.linalg.eigvals(self.A())
        real_vals = np.real(eigenvalues)
        imag_vals = np.imag(eigenvalues)

        fig = go.Figure()
        fig.add_trace(
            go.Scatter(
                x=real_vals,
                y=imag_vals,
                mode="markers",
                marker=dict(
                    size=12,
                    color="blue",
                    symbol="x",  # Use 'x' marker to match controls standards
                ),
                name="Eigenvalues",
            )
        )
        fig.update_layout(
            title="System Eigenvalues",
            xaxis_title="Real Part",
            yaxis_title="Imaginary Part",
            showlegend=True,
            width=600,
            height=400,
        )
        # Add some padding to the axis limits for better visualization
        pad_x = (
            (real_vals.max() - real_vals.min()) * 0.1
            if real_vals.max() != real_vals.min()
            else 1
        )
        pad_y = (
            (imag_vals.max() - imag_vals.min()) * 0.1
            if imag_vals.max() != imag_vals.min()
            else 1
        )
        fig.update_xaxes(range=[real_vals.min() - pad_x, real_vals.max() + pad_x])
        fig.update_yaxes(range=[imag_vals.min() - pad_y, imag_vals.max() + pad_y])
        return fig
