from dash import html, dcc
import plotly.graph_objs as go
import numpy as np


class BaseController:
    """A base class to define the basic interface for controllers"""

    title = None

    controller_inputs = {}

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
                            ),
                        ]
                    )
                    for name, props in cls.controller_inputs.items()
                ],
            ]
        )

    def initialize(self, dt: float):
        """Initialize the controller and set a time step.
        This method should be called before the first step.

        Args:
            dt: The time step for the controller.
        """
        raise NotImplementedError("This method should be implemented in subclasses")

    def step(self, y: float) -> float:
        """Calculate the control action based on the current output.
        Will also update the internal state of the controller.

        Args:
            output: The current output of the system (y).

        Returns:
            The control action (u) to be applied to the system.
        """
        raise NotImplementedError("This method should be implemented in subclasses")

    def make_analysis_plots(self, A: np.ndarray, B: np.ndarray, C:np.ndarray) -> list[go.Figure]:
        """
        Return a list of plot objects to be displayed when the inputs are changed for analysis.
        """
        return [
            self.mode_plot(A, B, C)
            ]
    
    def mode_plot(self, A: np.ndarray, B: np.ndarray, C: np.ndarray) -> go.Figure:
        """
        Return a plot of the controlled systems modes on the complex plane.
        """
        eigenvalues = self.calculate_controlled_eigenvalues(A, B, C)
        fig = go.Figure()
        fig.add_trace(
            go.Scatter(
                x=np.real(eigenvalues),
                y=np.imag(eigenvalues),
                mode="markers",
                marker=dict(size=10, color="blue"),
                name="Eigenvalues",
            )
        )
        fig.update_layout(
            title="Controlled System Eigenvalues",
            xaxis_title="Real Part",
            yaxis_title="Imaginary Part",
            showlegend=True,
            width=600,
            height=400,
        )
        fig.update_xaxes(range=[np.min(np.real(eigenvalues)), np.max(np.real(eigenvalues))])
        fig.update_yaxes(range=[np.min(np.imag(eigenvalues)), np.max(np.imag(eigenvalues))])
        return fig


    def calculate_controlled_eigenvalues(self, A: np.ndarray, B: np.ndarray, C: np.ndarray) -> np.ndarray:
        """
        Calculate the eigenvalues of the controlled system.
        """
        raise NotImplementedError("This method should be implemented in subclasses")
