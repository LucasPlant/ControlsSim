from dash import html, dcc
import numpy as np
import plotly.graph_objs as go
from plot_utils import MAX_PLOT_POINTS, get_plot_sample_indices

from .base_manipulator_system import BaseManipulatorSystem
from controllers import (
    BaseController,
    NoopController,
    PIDController,
    StateFeedbackController,
)

class CartPole(BaseManipulatorSystem):
    """A class representing a cart-pole system. Based on problem set 8 """

    title = "Cart-Pole System"

    # Controllers that are compatible with this system
    allowed_controllers = {
        "No Controller": NoopController,
        "PID Controller": PIDController,
        "State Feedback": StateFeedbackController,
    }

    system_args = {
        "mass_pole": {
            "type": "number",
            "value": 1.0,
            "description": "Mass of the pendulum (m)",
        },
        "mass_cart": {
            "type": "number",
            "value": 1.0,
            "description": "Mass of the cart (M)",
        },
        "length_pole": {
            "type": "number",
            "value": 1.0,
            "description": "Length of the pendulum (L)",
        },
        "gravity": {
            "type": "number",
            "value": 9.8,
            "description": "Acceleration due to gravity (m/s^2)",
        },
    }

    linearization_args = {
        "linearization_point_x": {
            "type": "number",
            "value": 0.0,
            "description": "Point around which to linearize the system (x position)",
        },
        "linearization_point_theta": {
            "type": "number",
            "value": np.pi,
            "description": "Point around which to linearize the system (theta) 0 is up π is down",
        },
        "linearization_point_x_dot": {
            "type": "number",
            "value": 0.0,
            "description": "Point around which to linearize the system (x velocity)",
        },
        "linearization_point_theta_dot": {
            "type": "number",
            "value": 0.0,
            "description": "Point around which to linearize the system (theta velocity)",
        },
        "linearization_control_force": {
            "type": "number",
            "value": 0.0,
            "description": "Control input around which to linearize the system (N*m)",
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
            "name": "x",
            "value": 0.2,
            "description": "Position of the cart (meters)",
        },
        {
            "name": "theta",
            "value": np.pi + 0.3,
            "description": "Position of the pendulum from down position (radians)",
            # Down is 0 and up is pi
        },
        {
            "name": "v",
            "value": 0.0,
            "description": "Velocity of the cart (m/s)",
        },
        {
            "name": "omega",
            "value": 0.0,
            "description": "Angular velocity of the pendulum (rad/s)",
        },
    ]

    output_info = ["X Position (x)", "Pendulum Angle (theta)"]

    input_info = ["Force on Cart (u)"]

    def __init__(
        self,
        mass_cart: float,
        mass_pole: float,
        length_pole: float,
        gravity: float,
        linearization_point_x: float,
        linearization_point_theta: float,
        linearization_control_force: float,
        linearization_point_x_dot: float,
        linearization_point_theta_dot: float,
        final_time: float,
        dt: float,
        controller_type: str,
        state_0: float,  # Initial position meters
        state_1: float,  # Initial position radians
        state_2: float,  # Initial velocity of cart m/s
        state_3: float,  # Initial angular velocity of pendulum radians/s
        controller_inputs: dict,
    ):
        """
        Initialize the mass-spring system with parameters and controller.

        Args:
            mass_cart: Mass of the cart.
            mass_pole: Mass of the pendulum.
            length_pole: Length of the pendulum.
            gravity: Acceleration due to gravity.

            final_time: Total simulation time.
            dt: Time step for the simulation.
            controller_type: The name of the controller to use.
            state_0: Initial position.
            state_1: Initial position radians.
            state_2: Initial velocity of cart m/s.
            state_3: Initial angular velocity of pendulum radians/s.
        """
        linearization_point = np.array([linearization_point_x, linearization_point_theta, linearization_point_x_dot, linearization_point_theta_dot])
        linearization_control = np.array([linearization_control_force])
        super().__init__(dt, final_time, controller_type, controller_inputs, linearization_point, linearization_control)
        self.mass_cart = mass_cart
        self.mass_pole = mass_pole
        self.length_pole = length_pole
        self.gravity = gravity
        self.initial_state = np.array([state_0, state_1, state_2, state_3], dtype=float)

    def M_manip(self, q: np.ndarray) -> np.ndarray:
        """The mass matrix M(q) of the system."""
        m_c = self.mass_cart
        m_p = self.mass_pole
        l = self.length_pole
        theta = q[1]

        M11 = m_c + m_p
        M12 = m_p * l * np.cos(theta)
        M21 = M12
        M22 = m_p * (l ** 2)

        return np.array([[M11, M12], [M21, M22]])
    
    def C_manip(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        """The Coriolis matrix C(q, qdot) of the system."""
        m_p = self.mass_pole
        l = self.length_pole
        theta = q[1]
        theta_dot = qdot[1]

        C11 = 0
        C12 = -m_p * l * theta_dot * np.sin(theta)
        C21 = 0
        C22 = 0

        return np.array([[C11, C12], [C21, C22]])
    
    def B_manip(self, q: np.ndarray) -> np.ndarray:
        """The input matrix B(q) of the system."""
        return np.array([[1], [0]])
    
    def tau(self, q: np.ndarray) -> np.ndarray:
        """The gravity term tau(q) of the system."""
        m_p = self.mass_pole
        l = self.length_pole
        g = self.gravity
        theta = q[1]

        tau_1 = 0
        tau_2 = -m_p * g * l * np.sin(theta)

        return np.array([tau_1, tau_2])
    
    def d_tau_dq(self, q: np.ndarray) -> np.ndarray:
        """The derivative of tau(q) with respect to q."""
        m_p = self.mass_pole
        l = self.length_pole
        g = self.gravity
        theta = q[1]

        d_tau_1_dq1 = 0
        d_tau_1_dq2 = 0
        d_tau_2_dq1 = 0
        d_tau_2_dq2 = m_p * g * l

        return np.array([[d_tau_1_dq1, d_tau_1_dq2], [d_tau_2_dq1, d_tau_2_dq2]])
    
    def d_Bj_dq(self, q: np.ndarray) -> np.ndarray:
        """The derivative of B(q) with respect to q."""
        return np.zeros((2, 1))

    def make_animation(self) -> go.Figure:
        """
        Create an animation of the cart-pole system.
        """
        cart_x = self.x[:, 0]
        theta = self.x[:, 1]
        cart_v = self.x[:, 2]
        theta_dot = self.x[:, 3]

        # Render convention: theta = 0 is down, theta = pi is up.
        pole_tip_x = cart_x + self.length_pole * np.sin(theta)

        cart_half_width = 0.25
        cart_height = 0.2
        cart_base_y = -cart_height / 2
        cart_top_y = cart_base_y + cart_height
        pole_tip_y = cart_top_y - self.length_pole * np.cos(theta)

        x_min = min(np.min(cart_x - cart_half_width), np.min(pole_tip_x)) - 0.5
        x_max = max(np.max(cart_x + cart_half_width), np.max(pole_tip_x)) + 0.5
        y_min = min(cart_base_y, np.min(pole_tip_y)) - 0.3
        y_max = max(cart_top_y, np.max(pole_tip_y)) + 0.3

        frame_indices = get_plot_sample_indices(len(self.t), MAX_PLOT_POINTS)

        track = go.Scatter(
            x=[x_min, x_max],
            y=[cart_base_y, cart_base_y],
            mode="lines",
            line=dict(color="gray", width=3),
            name="Track",
        )

        initial_idx = frame_indices[0]
        cart_box = go.Scatter(
            x=[
                cart_x[initial_idx] - cart_half_width,
                cart_x[initial_idx] + cart_half_width,
                cart_x[initial_idx] + cart_half_width,
                cart_x[initial_idx] - cart_half_width,
                cart_x[initial_idx] - cart_half_width,
            ],
            y=[cart_base_y, cart_base_y, cart_top_y, cart_top_y, cart_base_y],
            mode="lines",
            fill="toself",
            line=dict(color="royalblue", width=2),
            fillcolor="rgba(65, 105, 225, 0.35)",
            name="Cart",
        )

        pole = go.Scatter(
            x=[cart_x[initial_idx], pole_tip_x[initial_idx]],
            y=[cart_top_y, pole_tip_y[initial_idx]],
            mode="lines",
            line=dict(color="black", width=4),
            name="Pole",
        )

        bob = go.Scatter(
            x=[pole_tip_x[initial_idx]],
            y=[pole_tip_y[initial_idx]],
            mode="markers",
            marker=dict(size=14, color="crimson"),
            name="Pole Tip",
        )

        frames = []
        for i in frame_indices:
            frame_cart = go.Scatter(
                x=[
                    cart_x[i] - cart_half_width,
                    cart_x[i] + cart_half_width,
                    cart_x[i] + cart_half_width,
                    cart_x[i] - cart_half_width,
                    cart_x[i] - cart_half_width,
                ],
                y=[cart_base_y, cart_base_y, cart_top_y, cart_top_y, cart_base_y],
                mode="lines",
                fill="toself",
                line=dict(color="royalblue", width=2),
                fillcolor="rgba(65, 105, 225, 0.35)",
                name="Cart",
            )
            frame_pole = go.Scatter(
                x=[cart_x[i], pole_tip_x[i]],
                y=[cart_top_y, pole_tip_y[i]],
                mode="lines",
                line=dict(color="black", width=4),
                name="Pole",
            )
            frame_bob = go.Scatter(
                x=[pole_tip_x[i]],
                y=[pole_tip_y[i]],
                mode="markers",
                marker=dict(size=14, color="crimson"),
                name="Pole Tip",
            )
            frames.append(
                go.Frame(
                    data=[track, frame_cart, frame_pole, frame_bob],
                    name=str(i),
                    layout=go.Layout(
                        annotations=[
                            dict(
                                x=0.02,
                                y=0.98,
                                xref="paper",
                                yref="paper",
                                text=(
                                    f"Time: {self.t[i]:.2f}s<br>"
                                    f"x: {cart_x[i]:.2f} m<br>"
                                    f"theta: {theta[i]:.2f} rad<br>"
                                    f"v: {cart_v[i]:.2f} m/s<br>"
                                    f"omega: {theta_dot[i]:.2f} rad/s"
                                ),
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
            data=[track, cart_box, pole, bob],
            layout=go.Layout(
                title="Cart-Pole Animation",
                xaxis=dict(range=[x_min, x_max], title="X Position (m)"),
                yaxis=dict(range=[y_min, y_max], title="Y Position (m)"),
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
                            for i in frame_indices
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
