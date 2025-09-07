from .base_controller import BaseController
import numpy as np
import plotly.graph_objs as go


class PIDController(BaseController):
    """
    PID Controller.
    This controller implements a Proportional-Integral-Derivative (PID) control algorithm.
    The derivative portion is an approximation using a first order difference.
    TODO: Allow MIMO PID control
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

    state_info = [
        {
            "name": "Integrator",
            "value": 0.0,
            "description": "Integral of the error over time",
        },
        {
            "name": "Derivative",
            "value": 0.0,
            "description": "Derivative of the error (approximation)",
        },
    ]

    def __init__(self, y_target, Kp, Ki, Kd):
        super().__init__()
        self.y_target = y_target
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def initialize(self, A, B, C, dt, t, state_info):
        super().initialize(A, B, C, dt, t, state_info)

        self.integral = 0.0
        self.prev_error = 0.0

        # Initialize the state logging
        self.state = np.zeros((len(self.t), 2))  # Placeholder for state

    def step(self, y, index):
        error = self.y_target - y

        derivative = (error - self.prev_error) / self.dt
        if index == 0:
            derivative = 0.0
        self.prev_error = error

        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Update integral state
        self.integral += error * self.dt

        # Log the state
        self.state[index, 0] = self.integral
        self.state[index, 1] = derivative
        self.u[index] = u

        # Compute control input
        return np.array([u])

    def make_analysis_plots(self, A, B, C) -> list:
        return []
        # TODO make these plots work
        return [
            *super().make_analysis_plots(A, B, C),
            self.root_locus_plot(A, B, C),
        ]

    def calculate_controlled_eigenvalues(self, A, B, C) -> np.ndarray:
        return np.linalg.eigvals(A - B @ C)  # TODO this is inaccurate

    def root_locus_plot(self) -> go.Figure:
        # TODO
        raise NotImplementedError
