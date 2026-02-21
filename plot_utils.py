"""Basic re used plotting utils"""

import plotly.graph_objs as go
import numpy as np

MAX_PLOT_POINTS = 1000


def get_plot_sample_indices(num_points: int, max_points: int = MAX_PLOT_POINTS) -> np.ndarray:
    """Return evenly spaced indices capped at max_points."""
    if num_points <= 0:
        return np.array([], dtype=int)
    if num_points <= max_points:
        return np.arange(num_points, dtype=int)
    return np.linspace(0, num_points - 1, max_points, dtype=int)


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
    num_points = min(len(t), vars.shape[0])
    sample_indices = get_plot_sample_indices(num_points)
    t_plot = t[:num_points][sample_indices]
    vars_plot = vars[:num_points][sample_indices]

    state_plot = go.Figure()
    plot_colors = [
        "#636EFA",
        "#EF553B",
        "#00CC96",
        "#AB63FA",
        "#FFA15A",
        "#19D3F3",
        "#FF6692",
        "#B6E880",
    ]
    num_vars = vars_plot.shape[1]
    for i in range(num_vars):
        state_name = state_info[i]
        axis_name = "y" if i == 0 else f"y{i + 1}"
        state_plot.add_trace(
            go.Scatter(
                x=t_plot,
                y=vars_plot[:, i],
                mode="lines",
                name=state_name,
                yaxis=axis_name,
                line={"color": plot_colors[i % len(plot_colors)]},
            )
        )

    layout_update = {
        "title": title,
        "xaxis_title": "Time (s)",
        "legend_title": "Variables",
        "legend": {
            "orientation": "h",
            "yanchor": "bottom",
            "y": 1.02,
            "xanchor": "left",
            "x": 0.0,
        },
        "yaxis": {
            "showline": True,
            "linecolor": plot_colors[0],
            "tickfont": {"color": plot_colors[0]},
            "zeroline": False,
            "nticks": 5,
            "automargin": True,
        },
        "margin": {"t": 100, "l": 80, "r": 80},
    }

    # Give each line its own overlaid y-axis so traces with different magnitudes
    # remain readable while sharing the same time axis.
    if num_vars > 1:
        left_extra = sum(1 for i in range(1, num_vars) if i % 2 == 0)
        right_extra = sum(1 for i in range(1, num_vars) if i % 2 == 1)
        left_margin = 80 + 45 * left_extra
        right_margin = 80 + 45 * right_extra
        x_start = 0.08 + 0.06 * left_extra
        x_end = 0.92 - 0.06 * right_extra
        if x_end - x_start < 0.5:
            x_start, x_end = 0.25, 0.75

        left_axis_idx = 0
        right_axis_idx = 0
        for i in range(1, num_vars):
            axis_key = f"yaxis{i + 1}"
            is_right = (i % 2) == 1
            if is_right:
                right_axis_idx += 1
                axis_position = min(0.99, x_end + 0.05 * right_axis_idx)
            else:
                left_axis_idx += 1
                axis_position = max(0.01, x_start - 0.05 * left_axis_idx)

            layout_update[axis_key] = {
                "overlaying": "y",
                "side": "right" if is_right else "left",
                "anchor": "free",
                "position": axis_position,
                "showgrid": False,
                "showline": True,
                "linecolor": plot_colors[i % len(plot_colors)],
                "tickfont": {"color": plot_colors[i % len(plot_colors)]},
                "zeroline": False,
                "nticks": 5,
                "automargin": True,
            }

        state_plot.update_layout(
            xaxis={
                "domain": [x_start, x_end],
                "title": "Time (s)",
            },
            margin={"t": 100, "l": left_margin, "r": right_margin},
        )

    state_plot.update_layout(**layout_update)
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
