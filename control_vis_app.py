from dash import Dash, html, dcc, Input, Output, State
import dash
from dash.dependencies import ALL

from controllers import (
    BaseController,
    NoopController,
    PIDController,
    StateFeedbackController,
    StateFeedbackIntegralController,
)
from systems import BaseSystem, MassSpringSystem


SIM_OPTIONS: dict[str, type[BaseSystem]] = {
    "Mass-Spring System": MassSpringSystem,
}

CONTROLLER_OPTIONS: dict[str, type[BaseController]] = {
    "No Controller": NoopController,
    "PID Controller": PIDController,
    "State Feedback": StateFeedbackController,
    "State Feedback Integral": StateFeedbackIntegralController
}

app = Dash("Control Visualization App")

app.layout = html.Div(
    [
        html.H1("Control Visualization App"),
        html.Div(
            [
                # Left column: options and simulation controls
                html.Div(
                    [
                        html.H2("Select a System to Simulate"),
                        dcc.Dropdown(
                            id="system-selector",
                            options=[{"label": k, "value": k} for k in SIM_OPTIONS],
                            value=list(SIM_OPTIONS.keys())[0],
                        ),
                        html.H2("Select a Controller"),
                        dcc.Dropdown(
                            id="controller-selector",
                            options=[
                                {"label": k, "value": k} for k in CONTROLLER_OPTIONS
                            ],
                            value=list(CONTROLLER_OPTIONS.keys())[0],
                        ),
                        html.Div(id="system-inputs-container"),
                        html.Div(id="controller-inputs-container"),
                        html.Button("Run Simulation", id="run-btn"),
                    ],
                    style={"flex": "1", "minWidth": "350px", "marginRight": "40px"},
                ),
                # Right column: analysis plots
                html.Div(
                    id="analysis-plots-container",
                    style={"flex": "1", "minWidth": "350px"},
                ),
            ],
            style={
                "display": "flex",
                "flexDirection": "row",
                "alignItems": "flex-start",
            },
        ),
        html.Div(id="simulation-plots-container"),
        dcc.Store(id="system-input-store"),
        dcc.Store(id="controller-input-store"),
    ]
)


# Dynamically generate system input fields
@app.callback(
    Output("system-inputs-container", "children"),
    Input("system-selector", "value"),
)
def render_system_inputs(system_key):
    system_class = SIM_OPTIONS[system_key]
    return system_class.make_layout()


# Dynamically generate controller input fields
@app.callback(
    Output("controller-inputs-container", "children"),
    Input("controller-selector", "value"),
)
def render_controller_inputs(controller_key):
    controller_class = CONTROLLER_OPTIONS[controller_key]
    return controller_class.make_layout()


# Store system inputs as a json for kwargs
@app.callback(
    Output("system-input-store", "data"),
    Input({"type": "system-input", "name": ALL}, "value"),
    State({"type": "system-input", "name": ALL}, "id"),
)
def store_system_inputs(values, ids):
    return {id_["name"]: val for val, id_ in zip(values, ids)}


# Store controller inputs as a json for kwargs
@app.callback(
    Output("controller-input-store", "data"),
    Input({"type": "controller-input", "name": ALL}, "value"),
    State({"type": "controller-input", "name": ALL}, "id"),
)
def store_controller_inputs(values, ids):
    return {id_["name"]: val for val, id_ in zip(values, ids)}


# Run simulation
@app.callback(
    Output("simulation-plots-container", "children"),
    Input("run-btn", "n_clicks"),
    State("system-selector", "value"),
    State("controller-selector", "value"),
    State("system-input-store", "data"),
    State("controller-input-store", "data"),
)
def run_simulation(
    n_clicks, system_key, controller_key, system_inputs, controller_inputs
):
    if not n_clicks:
        return dash.no_update

    system_class = SIM_OPTIONS[system_key]
    controller_class = CONTROLLER_OPTIONS[controller_key]

    controller = controller_class(**controller_inputs)
    system = system_class(**system_inputs, controller=controller)

    system.simulate()

    return system.make_simulation_plots()


@app.callback(
    Output("analysis-plots-container", "children"),
    Input("system-input-store", "data"),
    Input("controller-input-store", "data"),
    State("system-selector", "value"),
    State("controller-selector", "value"),
)
def update_analysis_plots(system_inputs, controller_inputs, system_key, controller_key):
    if not system_inputs:
        return dash.no_update

    system_class = SIM_OPTIONS[system_key]
    controller_class = CONTROLLER_OPTIONS[controller_key]

    controller = controller_class(**controller_inputs)
    system = system_class(**system_inputs, controller=controller)

    # Assumes make_analysis_plots() returns Dash components
    return system.make_analysis_fields()


if __name__ == "__main__":
    app.run(debug=True)
