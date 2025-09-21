from dash import html, dcc
import numpy as np
import plotly.graph_objs as go

from .base_system import BaseSystem
from controllers import (
    BaseController,
    NoopController,
    PIDController,
    StateFeedbackController,
)

class MotorizedPendulum(BaseSystem):
    """A class representing a pendulum controlled by a motor."""

    title = "Pendulum on a Motor"

    # Controllers that are compatible with this system
    allowed_controllers = {
        "No Controller": NoopController,
        "PID Controller": PIDController,
        "State Feedback": StateFeedbackController,
    }

    system_args = {
        "mass": {
            "type": "number",
            "value": 1.0,
            "description": "Mass of the system (m)",
        },
        "length": {
            "type": "number",
            "value": 10.0,
            "description": "Length of the pendulum (L)",
        },
        "gravity": {
            "type": "number",
            "value": 9.8,
            "description": "Acceleration due to gravity (m/s^2)",
        },
        "linearization_point": {
            "type": "number",
            "value": np.pi,
            "description": "Point around which to linearize the system (radians) 0 is down pi is up",
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
            "name": "Position",
            "value": np.pi,
            "description": "Initial position of the pendulum (radians)",
        },
        {
            "name": "Velocity",
            "value": 0.0,
            "description": "Initial velocity of the pendulum (rad/s)",
        },
    ]

    output_info = ["Angle radians (y)"]

    input_info = ["Torque (u)"]

    def __init__(
        self,
        mass: float,
        length: float,
        gravity: float,
        linearization_point: float,
        final_time: float,
        dt: float,
        controller_type: str,
        state_0: float,  # Initial position radians
        state_1: float,  # Initial velocity radians/s
        controller_inputs: dict,
    ):
        """
        Initialize the mass-spring system with parameters and controller.

        Args:
            mass: Mass of the system.
            length: Length of the pendulum.
            g: Acceleration due to gravity.
            final_time: Total simulation time.
            dt: Time step for the simulation.
            controller_type: The name of the controller to use.
            state_0: Initial position.
            state_1: Initial velocity.
        """
        super().__init__(dt, final_time, controller_type, controller_inputs)
        self.mass = mass
        self.length = length
        self.gravity = gravity
        self.linearization_point = linearization_point
        self.initial_state = np.array([state_0, state_1], dtype=float)

    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        theta = x[0]
        theta_dot = x[1]
        u = u[0]  # Torque input

        theta_double_dot = (1/(self.mass * self.length**2) * u - 
                            (self.gravity / self.length) * np.sin(theta))
        
        return np.array([theta_dot, theta_double_dot])

    def g(self, x: np.ndarray) -> np.ndarray:
        return np.array([x[0]])

    def A(self) -> np.ndarray:
        linearization_point = np.array([self.linearization_point, 0])
        return np.array(
            [
                [0, 1],
                [
                    -(self.gravity / self.length) * np.cos(linearization_point[0]),
                    0
                ],
            ]
        )

    def B(self) -> np.ndarray:
        return np.array([[0], [1 / (self.mass * self.length**2)]])

    def C(self) -> np.ndarray:
        return np.array([[1, 0]])

    def make_animation(self) -> go.Figure:
        """
        Create an animation of the motorized pendulum showing position and velocity
        """
        # Extract angles and angular velocities
        theta = self.x[:, 0]  # Angle in radians
        theta_dot = self.x[:, 1]  # Angular velocity in rad/s
        
        # Convert to Cartesian coordinates for pendulum bob
        x_pos = self.length * np.sin(theta)
        y_pos = -self.length * np.cos(theta)  # Negative because we want down to be negative y
        
        # Calculate velocity components (tangential to pendulum path)
        x_vel = self.length * theta_dot * np.cos(theta)
        y_vel = self.length * theta_dot * np.sin(theta)
        
        # Calculate axis limits with padding
        axis_limit = self.length + 0.5
        
        # Scale factor for velocity arrows
        vel_scale = 0.3
        
        # Create pendulum rod (from pivot to bob) - initial position
        pendulum_rod = go.Scatter(
            x=[0, x_pos[0]],
            y=[0, y_pos[0]],
            mode="lines+markers",
            marker=dict(size=[8, 20], color=["black", "red"], 
                       symbol=["circle", "circle"]),
            line=dict(width=4, color="black"),
            name="Pendulum",
        )
        
        # Initial velocity arrow
        vel_magnitude = np.abs(theta_dot[0])
        if vel_magnitude > 0.01:  # Only show arrow if moving significantly
            vel_arrow = go.Scatter(
                x=[x_pos[0], x_pos[0] + x_vel[0] * vel_scale],
                y=[y_pos[0], y_pos[0] + y_vel[0] * vel_scale],
                mode="lines+markers",
                line=dict(color="green", width=3),
                marker=dict(size=[0, 8], color="green", symbol=["circle", "triangle-up"]),
                name="Velocity",
            )
        else:
            vel_arrow = go.Scatter(
                x=[x_pos[0]], y=[y_pos[0]],
                mode="markers", marker=dict(size=0, color="green"),
                name="Velocity",
            )

        # Create frames for animation
        frames = []
        for i in range(0, len(self.t), max(1, len(self.t) // 200)):  # Limit frames for performance
            # Pendulum rod for this frame
            pendulum_frame = go.Scatter(
                x=[0, x_pos[i]],
                y=[0, y_pos[i]],
                mode="lines+markers",
                marker=dict(size=[8, 20], color=["black", "red"], 
                           symbol=["circle", "circle"]),
                line=dict(width=4, color="black"),
                name="Pendulum",
            )
            
            # Velocity arrow for this frame
            vel_magnitude = np.abs(theta_dot[i])
            if vel_magnitude > 0.01:  # Only show arrow if moving significantly
                vel_arrow_frame = go.Scatter(
                    x=[x_pos[i], x_pos[i] + x_vel[i] * vel_scale],
                    y=[y_pos[i], y_pos[i] + y_vel[i] * vel_scale],
                    mode="lines+markers",
                    line=dict(color="green", width=3),
                    marker=dict(size=[0, 8], color="green", symbol=["circle", "triangle-up"]),
                    name="Velocity",
                )
            else:
                vel_arrow_frame = go.Scatter(
                    x=[x_pos[i]], y=[y_pos[i]],
                    mode="markers", marker=dict(size=0, color="green"),
                    name="Velocity",
                )

            frames.append(
                go.Frame(
                    data=[pendulum_frame, vel_arrow_frame],
                    name=str(i),
                    layout=go.Layout(
                        annotations=[
                            dict(
                                x=0.02,
                                y=0.98,
                                xref="paper",
                                yref="paper",
                                text=f"Time: {self.t[i]:.2f}s<br>Angle: {theta[i]:.2f} rad<br>ω: {theta_dot[i]:.2f} rad/s",
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

        fig = go.Figure(
            data=[pendulum_rod, vel_arrow],
            layout=go.Layout(
                title="Motorized Pendulum Animation",
                xaxis=dict(
                    range=[-axis_limit, axis_limit],
                    title="X Position (m)",
                    scaleanchor="y",
                    scaleratio=1,
                ),
                yaxis=dict(
                    range=[-axis_limit, axis_limit],
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
