from .base_controller import BaseController
import numpy as np
import plotly.graph_objs as go
import control
from scipy.signal import place_poles
from scipy.linalg import solve_continuous_lyapunov
from dash import html


class StateFeedbackController(BaseController):
    """
    A State Feedback controller that uses the linearized state space model and eigenvalues placement
    It uses a state estimator with eigenvalues lamda_e and a controller with eigenvalue lambda_c.
    """

    title = "State Feedback Controller"

    controller_inputs = {
        "lambda_e": {
            "type": "number",
            "value": -1.0,
            "description": "Estimator eigenvalue (lambda_e)",
        },
        "lambda_c": {
            "type": "number",
            "value": -1.0,
            "description": "Controller eigenvalue (lambda_c)",
        },
        "y_target": {
            "type": "number",
            "value": 1.0,
            "description": "Target position (y_target)",
        },
    }

    @staticmethod
    def calculate_gain_matrices(
        A: np.ndarray, B: np.ndarray, C: np.ndarray, lambda_c: float, lambda_e: float
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates the gain matrices currently with ackermann's formula however
        this should eventually support other methods like pole placement or LQR control

        Args:
        A: the state space A matrix
        B: the state space B matrix
        C: the state space C matrix
        lambda_c: the controller eigenvalues
        lambda_e: the estimator eigenvalues

        
        Returns:
        Controller and estimator gain matrices (K, L)
        """
        degree = A.shape[0]

        # Calculate gains using ackermans formula
        K = control.acker(A, B, [lambda_c] * degree)
        K = K.reshape(1, -1)
        L = control.acker(A.T, C.T, [lambda_e] * degree).T
        L = L.reshape(-1, 1)

        return K, L

    def __init__(self, y_target, lambda_e, lambda_c):
        self.lambda_e = lambda_e
        self.lambda_c = lambda_c
        self.y_target = y_target
        self.dt = None

        # Initialize the gain matrices
        self.K = np.array([])
        self.L = np.array([])

        # Initialize the ABC matrices
        self.A = np.array([])
        self.B = np.array([])
        self.C = np.array([])

    def initialize(self, A, B, C, dt, t, state_info):
        super().initialize(A, B, C, dt, t, state_info)

        self.state_info = state_info

        # Save the matricies
        self.A = A
        self.B = B
        self.C = C

        self.K, self.L = self.calculate_gain_matrices(
            self.A, self.B, self.C, self.lambda_c, self.lambda_e
        )

        degree = self.A.shape[0]
        # Initialize the estimated state array
        self.x_hat = np.zeros((len(self.t), degree))
        self.state = self.x_hat  # have the state pointer point at x_hat for plotting

    def step(self, y, index):
        x_hat = self.x_hat[index]

        # u = - k * x_hat
        u = -1 * (self.K @ x_hat)

        # x_hat_dot = A*x_hat + B*u - L(C*x_hat - y)
        # x_hat(index + 1) = x_hat + dt * x_hat_dot
        x_hat_dot = (self.A @ x_hat) + (self.B @ u) - self.L @ ((self.C @ x_hat) - y)
        self.x_hat[index + 1] = self.x_hat[index] + self.dt * x_hat_dot

        return u

    def make_analysis_plots(self, A, B, C) -> list:
        return [
            self.controllability_field(A, B, C),
            self.observability_field(A, B, C),
        ] + super().make_analysis_plots(A, B, C)

    def calculate_controlled_eigenvalues(self, A, B, C) -> np.ndarray:
        K, L = self.calculate_gain_matrices(A, B, C, self.lambda_c, self.lambda_e)

        # Commonly referenced matrix multiplications
        BK = B @ K
        LC = L @ C

        controller_eigenvalues = np.linalg.eigvals(A - BK)
        estimator_eigenvalues = np.linalg.eigvals(A - LC)

        return np.concat([controller_eigenvalues, estimator_eigenvalues])

    def controllability_field(
        self, A: np.ndarray, B: np.ndarray, C: np.ndarray
    ) -> html.H3:
        """
        Make a field for displaying the controllability of the system
        TODO make this more advanced with subspace plots ect

        Args:
        A: the state space A matrix
        B: the state space B matrix
        C: the state space C matrix

        Returns:
        A header with the return text
        """
        n = B.shape[0]
        controllability_matrix = control.ctrb(A, B)

        rank = np.linalg.matrix_rank(controllability_matrix)
        controllable = rank == n
        verb = "IS" if controllable else "ISNT"

        return html.H3(f"System {verb} controllable with rank {rank}\n")

    def observability_field(
        self, A: np.ndarray, B: np.ndarray, C: np.ndarray
    ) -> html.H3:
        """
        Make a field for displaying the observability of the system
        TODO make this more advanced with subspace plots ect

        Args:
        A: the state space A matrix
        B: the state space B matrix
        C: the state space C matrix

        Returns:
        A header with the return text
        """
        n = C.shape[1]
        observability_matrix = control.obsv(A, C)

        rank = np.linalg.matrix_rank(observability_matrix)
        observable = rank == n
        verb = "IS" if observable else "ISNT"

        return html.H3(f"System {verb} observable with rank {rank}\n")
