from numpy import ndarray
from plotly.graph_objs._figure import Figure
from .base_controller import BaseController


class NoopController(BaseController):
    """A controller that provides no control action."""

    title = "No Operation Controller"

    controller_inputs = {}

    def make_state_plots(self) -> list[Figure]:
        return []

    def make_analysis_plots(self, A: ndarray, B: ndarray, C: ndarray) -> list[Figure]:
        return []
