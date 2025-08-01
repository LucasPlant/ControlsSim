from .base_system import BaseSystem
import numpy as np


class BaseLinearSystem(BaseSystem):
    """A base class for linear systems, extending the BaseSystem class."""

    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        The state space model of the mass-spring system
        Defines x dot given x, u
        Change later if we want non time variant
        """
        return (self.A() @ x) + (self.B() @ u)

    def g(self, x: np.ndarray) -> np.ndarray:
        return self.C() @ x
