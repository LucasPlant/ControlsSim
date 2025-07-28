from dash import html, dcc
import numpy as np
import plotly.graph_objs as go

from .base_system import BaseSystem
from controllers import BaseController
from .base_linear_system import BaseLinearSystem


class MassSpringSystem(BaseLinearSystem):
    """A class representing a mass-spring system."""

    title = "Mass-Spring System"

    system_inputs = {
        "mass": {
            "type": "number",
            "value": 1.0,
            "description": "Mass of the system (m)",
        },
        "spring_constant": {
            "type": "number",
            "value": 10.0,
            "description": "Spring constant (k)",
        },
        "damping_coefficient": {
            "type": "number",
            "value": 0.0,
            "description": "Damping coefficient (c)",
        },
    }

    simulation_inputs = {
        "final_time": {
            "type": "number",
            "value": 10.0,
            "description": "Simulation time (s)",
        },
        "dt": {"type": "number", "value": 0.01, "description": "Time step (dt)"},
    }

    state_info = [
        {
            "name": "Position",
            "value": 0.0,
            "description": "Initial position of the mass (m)",
        },
        {
            "name": "Velocity",
            "value": 0.0,
            "description": "Initial velocity of the mass (m/s)",
        },
    ]

    def __init__(
        self,
        mass: float,
        spring_constant: float,
        damping_coefficient: float,
        final_time: float,
        dt: float,
        controller: BaseController,
        state_0: float,  # Initial position
        state_1: float,  # Initial velocity
    ):
        """
        Initialize the mass-spring system with parameters and a controller.

        Args:
            mass: Mass of the system.
            spring_constant: Spring constant of the system.
            damping_coefficient: Damping coefficient of the system.
            final_time: Total simulation time.
            dt: Time step for the simulation.
            controller: An instance of a controller that implements the BaseController interface.
        """
        super().__init__(controller, dt, final_time)
        self.mass = mass
        self.spring_constant = spring_constant
        self.damping_coefficient = damping_coefficient
        self.state = np.array([state_0, state_1], dtype=float)

    def A(self) -> np.ndarray:
        return np.array(
            [
                [0, 1],
                [
                    -self.spring_constant / self.mass,
                    -self.damping_coefficient / self.mass,
                ],
            ]
        )

    def B(self) -> np.ndarray:
        return np.array([[0, 1 / self.mass]])

    def C(self) -> np.ndarray:
        return np.array([[1, 0]])

    def make_animation(self) -> go.Figure:
        """
        Create an animation of the mass-spring system
        TODO this method may need some cleaning up
        """
        # Axis limits
        y_min = np.min(self.x[:, 0]) - 0.5
        y_max = np.max(self.x[:, 0]) + 0.5

        # Initial trace
        spring = go.Scatter(
            x=[0, self.x[0, 0]],
            y=[0, 0],
            mode="lines+markers",
            marker=dict(size=[1, 20]),
            line=dict(width=4),
        )

        # Frames for animation
        frames = [
            go.Frame(
                data=[
                    go.Scatter(
                        x=[0, self.x[i, 0]],
                        y=[0, 0],
                        mode="lines+markers",
                        marker=dict(size=[1, 20]),
                        line=dict(width=4),
                    )
                ],
                name=str(i),
                layout=go.Layout(
                    annotations=[
                        dict(
                            x=0.5,
                            y=1.05,
                            xref="paper",
                            yref="paper",
                            text=f"Sim Time: {self.t[i]:.2f}s",
                            showarrow=False,
                            font=dict(size=16),
                        )
                    ]
                ),
            )
            for i in range(len(self.t))
        ]

        fig = go.Figure(
            data=[spring],
            layout=go.Layout(
                title="Mass-Spring Animation",
                xaxis=dict(range=[y_min, y_max]),
                yaxis=dict(range=[-1, 1]),
                showlegend=False,
                updatemenus=[
                    {
                        "buttons": [
                            {
                                "args": [
                                    None,
                                    {
                                        "frame": {"duration": 100, "redraw": True},
                                        "fromcurrent": True,
                                    },
                                ],
                                "label": "Play",
                                "method": "animate",
                            },
                            {
                                "args": [
                                    [None],
                                    {
                                        "frame": {"duration": 0, "redraw": True},
                                        "mode": "immediate",
                                        "transition": {"duration": 0},
                                    },
                                ],
                                "label": "Pause",
                                "method": "animate",
                            },
                        ],
                        "direction": "left",
                        "pad": {"r": 10, "t": 87},
                        "showactive": False,
                        "type": "buttons",
                        "x": 0.1,
                        "xanchor": "right",
                        "y": 0,
                        "yanchor": "top",
                    }
                ],
                sliders=[
                    {
                        "steps": [
                            {
                                "args": [
                                    [str(i)],
                                    {
                                        "frame": {
                                            "duration": int(100 / 1.0),
                                            "redraw": True,
                                        },
                                        "mode": "immediate",
                                    },
                                ],
                                "label": f"{self.t[i]:.2f}s",
                                "method": "animate",
                            }
                            for i in range(len(self.t))
                        ],
                        "active": 0,
                        "yanchor": "top",
                        "xanchor": "left",
                        "currentvalue": {"prefix": "Sim Time: ", "font": {"size": 16}},
                        "transition": {"duration": 0, "easing": "cubic-in-out"},
                    }
                ],
            ),
            frames=frames,
        )
        return fig
