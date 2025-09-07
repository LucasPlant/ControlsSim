from .base_controller import BaseController
import numpy as np
import control
from scipy.signal import place_poles
from dash import html


class StateFeedbackIntegralController(BaseController):
    """
    A State Feedback controller that uses the linearized state space model and eigenvalues placement
    It uses a state estimator with eigenvalues lamda_e and a controller with eigenvalue lambda_c.
    It uses integral action to null steady state gain.
    """

    title = "State Feedback Controller with Integral Action"

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
        # Calculate the augmented state space matrices As and Bs
        # From A script and B script in the ECE4550 curriculum
        As = np.block(
            [
                [A, np.zeros((A.shape[0], B.shape[1]))],
                [C, np.zeros((C.shape[0], B.shape[1]))],
            ]
        )
        Bs = np.block([[B], [np.zeros((1, 1))]])

        # Calculate gains using ackermans formula
        controller_degree = As.shape[0]
        K = control.acker(As, Bs, [lambda_c] * controller_degree)
        K = K.reshape(1, -1)

        state_estimator_degree = A.shape[0]
        L = control.acker(A.T, C.T, [lambda_e] * state_estimator_degree).T
        L = L.reshape(-1, 1)

        return K, L

    def __init__(self, y_target, lambda_e, lambda_c):
        super().__init__()

        self.lambda_e = lambda_e
        self.lambda_c = lambda_c
        self.y_target = y_target

        # Initialize the gain matrices
        self.K1 = np.array([])
        self.K2 = np.array([])
        self.L = np.array([])

        # Initialize the ABC matrices
        self.A = np.array([])
        self.B = np.array([])
        self.C = np.array([])

    def initialize(self, A, B, C, dt, t, state_info):
        super().initialize(A, B, C, dt, t, state_info)

        self.state_info = state_info

        # Save the system matrices
        self.A = A
        self.B = B
        self.C = C

        K, self.L = self.calculate_gain_matrices(
            self.A, self.B, self.C, self.lambda_c, self.lambda_e
        )

        # Break out the gain matrix
        self.K1 = K[:, :-1]
        self.K1 = self.K1.reshape(1, -1)
        self.K2 = K[:, -1]
        self.K2 = self.K2.reshape(1, -1)

        degree = self.A.shape[0]
        # Initialize the estimated state array
        self.x_hat = np.zeros((len(self.t), degree))
        # TODO sigma going to need one sigma per output
        self.sigma = np.zeros((len(self.t), 1))
        self.state = self.x_hat  # have the state pointer point at x_hat for plotting

    def step(self, y, index):
        x_hat = self.x_hat[index, :]

        # u = - k1 * x_hat - k2 * sigma
        u = -1 * (self.K1 @ x_hat) + -1 * (self.K2 @ self.sigma[index, :])

        # x_hat_dot = A*x_hat + B*u - L(C*x_hat - y)
        # x_hat(index + 1) = x_hat + dt * x_hat_dot
        x_hat_dot = (self.A @ x_hat) + (self.B @ u) - self.L @ ((self.C @ x_hat) - y)
        self.x_hat[index + 1, :] = self.x_hat[index, :] + self.dt * x_hat_dot

        # update the integral term sigma
        # sigma(i+1) = sigma + dt * error
        self.sigma[index + 1, :] = self.sigma[index, :] + self.dt * (y - self.y_target)

        return u

    # TODO These should be made static and possibly moved out of here
    def make_analysis_plots(self, A, B, C) -> list:
        return [
            self.controllability_field(A, B, C),
            self.observability_field(A, B, C),
            self.commandability_field(A, B, C),
        ] + super().make_analysis_plots(A, B, C)

    def calculate_controlled_eigenvalues(self, A, B, C) -> np.ndarray:
        K, L = self.calculate_gain_matrices(A, B, C, self.lambda_c, self.lambda_e)

        # Break out the gain matrix
        K1 = K[:, :-1]
        K1 = K1.reshape(1, -1)
        K2 = K[:, -1]
        K2 = K2.reshape(1, -1)

        BK1 = B @ K1
        BK2 = B @ K2
        LC = L @ C

        full_system = np.block(
            [
                [(A - BK1), -BK2, -BK1],
                [
                    C,
                    np.zeros((C.shape[0], BK2.shape[1])),
                    np.zeros((C.shape[0], BK1.shape[1])),
                ],
                [
                    np.zeros((A.shape[0], A.shape[1])),
                    np.zeros((A.shape[0], BK2.shape[1])),
                    (A - LC),
                ],
            ]
        )

        return np.linalg.eigvals(full_system)

    # TODO can these be placed into a math or plotting file for reuse
    def commandability_field(
        self, A: np.ndarray, B: np.ndarray, C: np.ndarray
    ) -> html.H3:
        """
        Make a field for displaying the commandability of the system
        Commandability is the ability to command the output to any scalar value at steady state

        Args:
        A: the state space A matrix
        B: the state space B matrix
        C: the state space C matrix

        Returns:
        A header with the return text
        """
        commandability_matrix = np.block(
            [
                [A, B],
                [C, np.zeros((C.shape[0], B.shape[1]))],
            ]
        )

        commendable = np.linalg.det(commandability_matrix) != 0

        verb = "IS" if commendable else "ISNT"

        return html.H3(f"System {verb} commendable")

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
