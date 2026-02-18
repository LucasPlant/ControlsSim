from dash import html, dcc
import numpy as np
import plotly.graph_objs as go
from plot_utils import MAX_PLOT_POINTS, get_plot_sample_indices

from .base_system import BaseSystem
from controllers import (
    BaseController,
    NoopController,
    PIDController,
    StateFeedbackController,
)
from .base_linear_system import BaseLinearSystem


class TurtleBot(BaseLinearSystem):
    """A class representing a turtle bot system or a free moving mass in 2D."""

    title = "Turtle Bot System"

    # Controllers that are compatible with this system
    allowed_controllers = {
        "No Controller": NoopController,
        # "PID Controller": PIDController,
        "State Feedback": StateFeedbackController,
    }

    system_args = {
        "mass": {
            "type": "number",
            "value": 1.0,
            "description": "Mass of the system (m)",
        },
    }

    simulation_args = {
        "final_time": {
            "type": "number",
            "value": 10.0,
            "description": "Simulation time (s)",
        },
        "dt": {"type": "number", "value": 0.01, "description": "Time step (dt)"},
    }

    state_info = [
        {
            "name": "X Position",
            "value": 0.0,
            "description": "Initial position of the bot in x (m)",
        },
        {
            "name": "Y Position",
            "value": 0.0,
            "description": "Initial position of the bot in y (m)",
        },
        {
            "name": "X Velocity",
            "value": 0.0,
            "description": "Initial velocity of the bot in x (m/s)",
        },
        {
            "name": "Y Velocity",
            "value": 0.0,
            "description": "Initial velocity of the bot in y (m/s)",
        },
    ]

    output_info = ["X Position (y1)", "Y Position (y2)"]

    input_info = ["Force (ux)", "Force (uy)"]

    def __init__(
        self,
        mass: float,
        final_time: float,
        dt: float,
        controller_type: str,
        state_0: float = 0.0,  # Initial position x
        state_1: float = 0.0,  # Initial position y
        state_2: float = 0.0,  # Initial velocity x
        state_3: float = 0.0,  # Initial velocity y
        **kwargs,
    ):
        """
        Initialize the turtle bot system with parameters and controller.

        Args:
            mass: Mass of the system.
            final_time: Total simulation time.
            dt: Time step for the simulation.
            controller_type: The name of the controller to use.
            state_0: Initial position x.
            state_1: Initial position y.
            state_2: Initial velocity x.
            state_3: Initial velocity y.
            **kwargs: Additional arguments including controller parameters.
        """
        super().__init__(dt, final_time, controller_type, **kwargs)
        self.mass = mass
        self.initial_state = np.array([state_0, state_1, state_2, state_3], dtype=float)

    def A(self) -> np.ndarray:
        return np.array(
            [
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
            ]
        )

    def B(self) -> np.ndarray:
        return np.array([[0, 0], [0, 0], [1 / self.mass, 0], [0, 1 / self.mass]])

    def C(self) -> np.ndarray:
        return np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

    def make_animation(self) -> go.Figure:
        """
        Create an animation of the TurtleBot moving in 2D space with velocity arrows
        TODO may want to check this but it looks good
        """
        # Extract positions and velocities
        x_pos = self.x[:, 0]  # X position
        y_pos = self.x[:, 1]  # Y position
        x_vel = self.x[:, 2]  # X velocity
        y_vel = self.x[:, 3]  # Y velocity

        # Calculate axis limits with some padding
        x_min, x_max = np.min(x_pos) - 1, np.max(x_pos) + 1
        y_min, y_max = np.min(y_pos) - 1, np.max(y_pos) + 1

        # Scale factor for velocity arrows (adjust as needed)
        vel_scale = 0.5
        trajectory_indices = get_plot_sample_indices(len(self.t), MAX_PLOT_POINTS)

        # Create trajectory trace (shows full path)
        trajectory = go.Scatter(
            x=x_pos[trajectory_indices],
            y=y_pos[trajectory_indices],
            mode="lines",
            line=dict(color="lightblue", width=2, dash="dash"),
            name="Trajectory",
            opacity=0.6,
        )

        # Initial position and velocity arrow
        current_pos = go.Scatter(
            x=[x_pos[0]],
            y=[y_pos[0]],
            mode="markers",
            marker=dict(size=15, color="red", symbol="circle"),
            name="TurtleBot",
        )

        # Initial velocity arrow
        vel_arrow = go.Scatter(
            x=[x_pos[0], x_pos[0] + x_vel[0] * vel_scale],
            y=[y_pos[0], y_pos[0] + y_vel[0] * vel_scale],
            mode="lines+markers",
            line=dict(color="green", width=3),
            marker=dict(size=[0, 8], color="green", symbol=["circle", "triangle-up"]),
            name="Velocity",
        )

        # Create frames for animation
        frames = []
        for i in range(
            0, len(self.t), max(1, len(self.t) // 200)
        ):  # Limit frames for performance
            # Current position
            current_frame_pos = go.Scatter(
                x=[x_pos[i]],
                y=[y_pos[i]],
                mode="markers",
                marker=dict(size=15, color="red", symbol="circle"),
                name="TurtleBot",
            )

            # Velocity arrow
            vel_magnitude = np.sqrt(x_vel[i] ** 2 + y_vel[i] ** 2)
            if vel_magnitude > 0.01:  # Only show arrow if moving significantly
                vel_arrow_frame = go.Scatter(
                    x=[x_pos[i], x_pos[i] + x_vel[i] * vel_scale],
                    y=[y_pos[i], y_pos[i] + y_vel[i] * vel_scale],
                    mode="lines+markers",
                    line=dict(color="green", width=3),
                    marker=dict(
                        size=[0, 8], color="green", symbol=["circle", "triangle-up"]
                    ),
                    name="Velocity",
                )
            else:
                vel_arrow_frame = go.Scatter(
                    x=[x_pos[i]],
                    y=[y_pos[i]],
                    mode="markers",
                    marker=dict(size=0, color="green"),
                    name="Velocity",
                )

            frames.append(
                go.Frame(
                    data=[trajectory, current_frame_pos, vel_arrow_frame],
                    name=str(i),
                    layout=go.Layout(
                        annotations=[
                            dict(
                                x=0.02,
                                y=0.98,
                                xref="paper",
                                yref="paper",
                                text=f"Time: {self.t[i]:.2f}s<br>Pos: ({x_pos[i]:.2f}, {y_pos[i]:.2f})<br>Vel: ({x_vel[i]:.2f}, {y_vel[i]:.2f})",
                                showarrow=False,
                                font=dict(size=12),
                                bgcolor="rgba(255,255,255,0.8)",
                                bordercolor="black",
                                borderwidth=1,
                                align="left",
                            )
                        ]
                    ),
                )
            )

        # Create the figure
        fig = go.Figure(
            data=[trajectory, current_pos, vel_arrow],
            layout=go.Layout(
                title="TurtleBot 2D Motion Animation",
                xaxis=dict(
                    range=[x_min, x_max],
                    title="X Position (m)",
                    scaleanchor="y",
                    scaleratio=1,
                ),
                yaxis=dict(
                    range=[y_min, y_max],
                    title="Y Position (m)",
                ),
                showlegend=True,
                legend=dict(x=0.02, y=0.02),
                updatemenus=[
                    {
                        "buttons": [
                            {
                                "args": [
                                    None,
                                    {
                                        "frame": {"duration": 50, "redraw": True},
                                        "fromcurrent": True,
                                    },
                                ],
                                "label": "▶ Play",
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
                                "label": "⏸ Pause",
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
                                        "frame": {"duration": 50, "redraw": True},
                                        "mode": "immediate",
                                    },
                                ],
                                "label": f"{self.t[i]:.1f}s",
                                "method": "animate",
                            }
                            for i in range(0, len(self.t), max(1, len(self.t) // 200))
                        ],
                        "active": 0,
                        "yanchor": "top",
                        "xanchor": "left",
                        "currentvalue": {"prefix": "Time: ", "font": {"size": 14}},
                        "transition": {"duration": 0, "easing": "cubic-in-out"},
                        "len": 0.9,
                        "x": 0.1,
                        "y": 0,
                    }
                ],
            ),
            frames=frames,
        )

        return fig
