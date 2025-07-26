from .base_controller import BaseController
import numpy as np
import plotly.graph_objs as go


class PIDController(BaseController):
    """
    PID Controller.
    This controller implements a Proportional-Integral-Derivative (PID) control algorithm.
    The derivative portion is an approximation using a first order difference.
    """

    title = "PID Controller"

    controller_inputs = {
        "Kp": {
            "type": "number",
            "value": 50.0,
            "description": "Proportional gain (Kp)",
        },
        "Ki": {"type": "number", "value": 0.0, "description": "Integral gain (Ki)"},
        "Kd": {"type": "number", "value": 5.0, "description": "Derivative gain (Kd)"},
        "y_target": {
            "type": "number",
            "value": 1.0,
            "description": "Target position (y_target)",
        },
    }

    def __init__(self, y_target, Kp, Ki, Kd):
        super().__init__()
        self.y_target = y_target
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = None

    def initialize(self, A, B, C, dt: float, t: np.ndarray):
        super().initialize(A, B, C, dt, t)

        self.integral = 0.0
        self.prev_error = 0.0

        # Initialize the state logging
        self.state = np.zeros((len(t), 2))  # Placeholder for state,

    def step(self, y: float, index: int) -> float:
        error = self.y_target - y

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Update integral state
        self.integral += error * self.dt

        # Log the state
        self.state[index, 0] = self.integral
        self.state[index, 1] = derivative
        self.u[index] = u

        # Compute control input
        return u

    def make_analysis_plots(
        self, A: np.ndarray, B: np.ndarray, C: np.ndarray
    ) -> list[go.Figure]:
        return []
        return [
            *super().make_analysis_plots(A, B, C),
            self.root_locus_plot(A, B, C),
        ]

    def calculate_controlled_eigenvalues(
        self, A: np.ndarray, B: np.ndarray, C: np.ndarray
    ) -> np.ndarray:
        return np.linalg.eigvals(A - B @ C)

    def root_locus_plot(self, A: np.ndarray, B: np.ndarray, C: np.ndarray) -> go.Figure:
        """
        TODO made by AI so check this
        Create a root locus plot for the PID controller.
        """
        # Placeholder for actual root locus computation
        # This should compute the poles of the closed-loop system
        # and plot them in the complex plane.

        fig = go.Figure()
        fig.update_layout(
            title="Root Locus Plot", xaxis_title="Real", yaxis_title="Imaginary"
        )
        return fig
