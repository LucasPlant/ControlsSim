from .base_system import BaseSystem
import numpy as np


class BaseManipulatorSystem(BaseSystem):
    """
    A base class for systems in basic manipulator form, extending the BaseSystem class.
    Math comes from the underactuated robotics course by Russ Tedrake, specifically the section on manipulator equations.
    https://underactuated.csail.mit.edu/acrobot.html#cart_pole.
    """

    def __init__(self,
                    dt: float,
                    final_time: float,
                    controller_type: str,
                    controller_inputs: dict,
                    linearization_point: np.ndarray,
                    linearization_control: np.ndarray
                ):
        super().__init__(dt, final_time, controller_type, controller_inputs)
        self.linearization_point = linearization_point
        self.linearization_control = linearization_control

    def x_to_q(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Convert from state space representation to configuration space representation."""
        half = len(x) // 2
        return x[:half], x[half:]
    
    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        The state space model of the system in manipulator form.

        Args:
            x: The current state of the system, where the first half is the configuration q and the second half is the velocity qdot.
            u: The control input to the system.
        Returns:
            The derivative of the state xdot, where the first half is qdot and the second half is qddot.
        """
        q, qdot = self.x_to_q(x)
        M_manip = self.M_manip(q)
        C_manip = self.C_manip(q, qdot)
        B_manip = self.B_manip(q)
        tau = self.tau(q)

        q_ddot = np.linalg.inv(M_manip) @ (tau + B_manip @ u - C_manip @ qdot)
        return np.concatenate((qdot, q_ddot))
    
    def g(self, x: np.ndarray) -> np.ndarray:
        """Default output is just q, but can be overridden by subclasses."""
        q, _ = self.x_to_q(x)
        return q
    
    def A(self) -> np.ndarray:
        """The linearized state matrix A of the system."""
        q, qdot = self.x_to_q(self.linearization_point)
        M_manip = self.M_manip(q)
        M_inv = np.linalg.inv(M_manip)
        C_manip = self.C_manip(q, qdot)
        d_tau_dq = self.d_tau_dq(q)
        d_Bj_dq = self.d_Bj_dq(q)

        sum = np.zeros_like(M_manip)
        for j in range(len(self.linearization_control)):
            sum += M_inv @ d_Bj_dq[:, j] * self.linearization_control[j]
        
        M21 = np.linalg.inv(M_manip) @ d_tau_dq + sum
        return np.block([
            [np.zeros((len(q), len(q))), np.eye(len(q))],
            [M21, np.zeros((len(q), len(q)))]
        ])
    
    def B(self) -> np.ndarray:
        """The linearized input matrix B of the system."""
        q, _ = self.x_to_q(self.linearization_point)
        M_manip = self.M_manip(q)
        B_manip = self.B_manip(q)

        return np.block([
            [np.zeros(B_manip.shape)],
            [np.linalg.inv(M_manip) @ B_manip]
        ])
    
    def C(self) -> np.ndarray:
        """The linearized output matrix C of the system."""
        q, _ = self.x_to_q(self.linearization_point)
        return np.block([
            [np.eye(len(q)), np.zeros((len(q), len(q)))]
        ])
    
    def M_manip(self, q: np.ndarray) -> np.ndarray:
        """The mass matrix of the manipulator."""
        raise NotImplementedError("Subclasses must implement the M_manip method.")
    
    def C_manip(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        """The Coriolis and centrifugal matrix of the manipulator."""
        raise NotImplementedError("Subclasses must implement the C_manip method.")
    
    def B_manip(self, q: np.ndarray) -> np.ndarray:
        """The input matrix of the manipulator."""
        raise NotImplementedError("Subclasses must implement the B_manip method.")
    
    def tau(self, q: np.ndarray) -> np.ndarray:
        """The gravity vector of the manipulator."""
        raise NotImplementedError("Subclasses must implement the tau method.")
    
    def d_tau_dq(self, q: np.ndarray) -> np.ndarray:
        """The derivative of the gravity vector with respect to q."""
        raise NotImplementedError("Subclasses must implement the d_tau_dq method.")
    
    def d_Bj_dq(self, q: np.ndarray) -> np.ndarray:
        """The derivative of the input matrix with respect to q evaluated at point q."""
        raise NotImplementedError("Subclasses must implement the d_B_dq method.")