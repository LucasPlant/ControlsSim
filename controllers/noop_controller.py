from .base_controller import BaseController


class NoopController(BaseController):
    """A controller that provides no control action."""

    title = "No Operation Controller"

    controller_inputs = {}

    def initialize(self, dt: float):
        """Initialize the no-op controller."""
        pass

    def step(self, y: float) -> float:
        """Return zero control action."""
        return 0.0
    
    def make_analysis_plots(self, A, B, C):
        """
        Return an empty list of plots since this controller does not perform any control.
        """
        return []
