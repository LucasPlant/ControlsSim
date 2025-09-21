from .base_controller import BaseController
import numpy as np
import control
from scipy.signal import place_poles
from dash import html


class StateFeedbackController(BaseController):
    """
    A State Feedback controller that uses the linearized state space model and eigenvalues placement
    It uses a state estimator with eigenvalues lamda_e and a controller with eigenvalue lambda_c.
    """

    title = "State Feedback Controller"

    controller_inputs = {
        "feedback_type": {
            "type": "dropdown",
            "value": "Integral Pole Placement",
            "description": "Controller design method",
            "options": [
                "Pole Placement",
                "Integral Pole Placement",
                # "LQR",  # LQR not currently supported
            ],
        },
        "state_estimator_type": {
            "type": "dropdown",
            "value": "Pole Placement",
            "description": "State estimator design method",
            "options": [
                "Pole Placement",
                # "Kalman Filter",  # Kalman filter not currently supported
            ],
        },
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
    }

    def __init__(
        self, lambda_e, lambda_c, feedback_type, state_estimator_type, trajectory_generator, trajectory_generator_inputs
    ):
        super().__init__(trajectory_generator, trajectory_generator_inputs)

        self.lambda_e = lambda_e
        self.lambda_c = lambda_c
        self.feedback_type = feedback_type  # Map from UI parameter name
        self.state_estimator_type = state_estimator_type

        # Initialize the gain matrices
        self.K = np.array([])
        self.L = np.array([])

        # Initialize the system matrices
        self.A = np.array([])
        self.B = np.array([])
        self.C = np.array([])

        # Initialize the gain arrays
        self.K = np.array([])
        self.L = np.array([])
        self.K1 = np.array([])

    def initialize(self, A, B, C, dt, t, state_info, output_info):
        super().initialize(A, B, C, dt, t, state_info, output_info)

        # Store the system matrices
        self.A = A
        self.B = B
        self.C = C

        # Calculate the gain matrices based on the selected method
        self.calculate_gain_matrices()

        # Initialize the estimated state array
        degree = self.A.shape[0]
        self.x_hat = np.zeros((len(self.t), degree))
        self.sigma = np.zeros((len(self.t), self.C.shape[0]))
        self.state = self.x_hat  # have the state pointer point at x_hat for plotting

    def step(self, y, index):
        x_hat = self.x_hat[index, :]

        if self.feedback_type == "Integral Pole Placement":
            # u = - k1 * x_hat - k2 * sigma
            u = -1 * (self.K1 @ x_hat) + -1 * (self.K2 @ self.sigma[index, :])
        else:
            # u = - k * x_hat
            u = -1 * (self.K @ x_hat)

        # x_hat_dot = A*x_hat + B*u - L(C*x_hat - y)
        # x_hat(index + 1) = x_hat + dt * x_hat_dot
        x_hat_dot = (self.A @ x_hat) + (self.B @ u) - self.L @ ((self.C @ x_hat) - y)
        self.x_hat[index + 1, :] = self.x_hat[index, :] + self.dt * x_hat_dot

        # update the integral term sigma
        # sigma(i+1) = sigma + dt * error
        self.sigma[index + 1, :] = self.sigma[index, :] + self.dt * (y - self.reference_trajectory[index])

        return u

    def make_analysis_plots(self, A, B, C) -> list:
        return [
            self.controllability_field(A, B, C),
            self.observability_field(A, B, C),
            self.commandability_field(A, B, C),
        ] + super().make_analysis_plots(A, B, C)

    def calculate_gain_matrices(self):
        """
        Calculates the gain matrices using the specified controller design method

        Returns:
        Controller and estimator gain matrices (K, L)
        """
        if self.feedback_type == "Pole Placement":
            self.K = self._calculate_gain_matrices_pole_placement(
                self.A, self.B, self.lambda_c
            )
        elif self.feedback_type == "Integral Pole Placement":
            # Calculate the augmented state space matrices As and Bs
            # From A script and B script in the ECE4550 curriculum
            self.As = np.block(
                [
                    [self.A, np.zeros((self.A.shape[0], self.B.shape[1]))],
                    [self.C, np.zeros((self.C.shape[0], self.B.shape[1]))],
                ]
            )
            self.Bs = np.block([[self.B], [np.zeros((self.B.shape[1], self.C.shape[0]))]])
            K = self._calculate_gain_matrices_pole_placement(
                self.As, self.Bs, self.lambda_c
            )

            # Break out the gain matrices
            # the dimensions should match u = -K1*x (u, n)
            # and xdot = B * u
            # for the dimensions to work out K1 should be (m, n)
            # Where B is (n, m) and A is (n, n)
            self.K1 = K[:, :-self.C.shape[0]]
            self.K1 = self.K1.reshape(self.B.shape[1], self.A.shape[0])

            # the dimensions should match u = -K2*sigma (u, p)
            # There is one sigma is a column vector for each output
            # for the dimensions to work out K2 should be (m, p)
            # Where B is (n, m) and C is (p, n)
            self.K2 = K[:, -self.C.shape[0]:]
            self.K2 = self.K2.reshape(self.B.shape[1], self.C.shape[0])
        else:
            raise ValueError(
                f"Unknown controller design method: {self.feedback_type}"
            )

        if self.state_estimator_type == "Pole Placement":
            # Use pole placement to calculate the L matrix as well
            self.L = self._calculate_gain_matrices_pole_placement(
                self.A.T, self.C.T, self.lambda_c
            ).T
        else:
            raise ValueError(
                f"Unknown state estimator design method: {self.state_estimator_type}"
            )

    def _calculate_gain_matrices_pole_placement(
        self, A, B, pole_position
    ) -> np.ndarray:
        """
        Calculate gain matrices using standard pole placement

        Returns:
        Controller and estimator gain matrices (K, L)
        """
        degree = A.shape[0]

        # Create eigenvalue arrays with slight variations to avoid repeated poles
        # The numerical method used in place_poles can struggle with repeated poles so we perturb them slightly
        lambdas = [pole_position * (1 + 1e-3 * i) for i in range(degree)]

        # Calculate controller gain matrix K
        gains = place_poles(A, B, lambdas).gain_matrix
        gains = gains.reshape(B.shape[1], degree)

        return gains

    def calculate_controlled_eigenvalues(self, A, B, C) -> np.ndarray:
        self.A = A
        self.B = B
        self.C = C

        self.calculate_gain_matrices()

        if self.feedback_type == "Integral Pole Placement":
            BK1 = B @ self.K1
            BK2 = B @ self.K2
            LC = self.L @ C

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
            eigenvalues = np.linalg.eigvals(full_system)

        else:
            # Commonly referenced matrix multiplications
            BK = B @ self.K
            LC = self.L @ C

            controller_eigenvalues = np.linalg.eigvals(A - BK)
            estimator_eigenvalues = np.linalg.eigvals(A - LC)
            eigenvalues = np.concat([controller_eigenvalues, estimator_eigenvalues])

        return eigenvalues

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

        commandable = np.linalg.det(commandability_matrix) != 0

        verb = "IS" if commandable else "ISNT"

        return html.H3(f"System {verb} commandable")
