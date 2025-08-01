from numpy import ndarray
import plotly.graph_objs as go
from .base_controller import BaseController


class NoopController(BaseController):
    """A controller that provides no control action."""

    title = "No Operation Controller"

    controller_inputs = {}

    def make_state_plots(self) -> list[go.Figure]:
        return []

    def make_analysis_plots(self, A, B, C) -> list[go.Figure]:
        return []
