"""Basic re used plotting utils"""

import plotly.graph_objs as go
import numpy as np


def multivar_plot(
    vars: np.ndarray, t: np.ndarray, state_info: list[str], title: str
) -> go.Figure:
    """
    A utility to easily plot multiple vars on the same axes good for state plots
    TODO Look into replacing this with dataframe based plotting for ease
    
    Args:
    vars: numpy array containing the vars over time to plot (time, var)
    t: the numpy array of timestamps
    state_info: list containing labels for the variables for the legend
    title: the title of the plot
    """
    state_plot = go.Figure()
    num_vars = vars.shape[1]
    for i in range(num_vars):
        state_name = state_info[i]
        state_plot.add_trace(
            go.Scatter(
                x=t,
                y=vars[:, i],
                mode="lines",
                name=state_name,
            )
        )
    state_plot.update_layout(
        title=title,
        xaxis_title="Time (s)",
        yaxis_title="Var Value",
        legend_title="Variables",
    )
    return state_plot


def mode_plot(eigenvalues: np.ndarray, title: str) -> go.Figure:
    """
    Make a plot of the modes given the eigenvalues

    Args:
    eigenvalues: a ndarray containing the complex eigenvalues of the system
    title: the title of the plot

    Returns:
    the figure containing the plot
    """
    real_vals = np.real(eigenvalues)
    imag_vals = np.imag(eigenvalues)

    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=real_vals,
            y=imag_vals,
            mode="markers",
            marker=dict(
                size=12,
                color="blue",
                symbol="x",  # Use 'x' marker to match controls standards
            ),
            name="Eigenvalues",
        )
    )
    fig.update_layout(
        title=title,
        xaxis_title="Real Part",
        yaxis_title="Imaginary Part",
        showlegend=True,
        width=600,
        height=400,
    )
    # Add some padding to the axis limits for better visualization
    pad_x = (
        (real_vals.max() - real_vals.min()) * 0.1
        if real_vals.max() != real_vals.min()
        else 1
    )
    pad_y = (
        (imag_vals.max() - imag_vals.min()) * 0.1
        if imag_vals.max() != imag_vals.min()
        else 1
    )
    fig.update_xaxes(range=[real_vals.min() - pad_x, real_vals.max() + pad_x])
    fig.update_yaxes(range=[imag_vals.min() - pad_y, imag_vals.max() + pad_y])
    return fig
