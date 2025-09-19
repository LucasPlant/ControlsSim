from dash import Dash, html, dcc, Input, Output, State
import dash
from dash.dependencies import ALL

from systems import BaseSystem, MassSpringSystem, TurtleBot

LOCAL = False

SIM_OPTIONS: dict[str, type[BaseSystem]] = {
    "Mass-Spring System": MassSpringSystem,
    "TurtleBot": TurtleBot,
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
                        html.Div(id="system-inputs-container"),
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


# Dynamically generate controller input fields when controller selection changes
@app.callback(
    Output({"type": "controller-inputs", "system": ALL}, "children"),
    Input({"type": "input", "source": "system", "name": "controller_type"}, "value"),
    State("system-selector", "value"),
    State("system-input-store", "data"),
)
def render_controller_inputs(controller_type, system_key, inputs):
    if not controller_type or not system_key:
        return [html.Div()]

    system_class = SIM_OPTIONS[system_key]
    controller_class = system_class.allowed_controllers.get(controller_type)

    if not controller_class:
        return [html.Div()]

    # Get existing controller inputs if they exist
    controller_inputs = {}
    if inputs and "controller" in inputs:
        controller_inputs = inputs["controller"]

    return [controller_class.make_layout(controller_inputs)]


# Store all inputs and sort between controller and system inputs
@app.callback(
    Output("system-input-store", "data"),
    Input({"type": "input", "source": ALL, "name": ALL}, "value"),
    State({"type": "input", "source": ALL, "name": ALL}, "id"),
)
def store_inputs(values, ids):
    # Separate system and controller inputs
    system_inputs = {}
    controller_inputs = {}

    for val, id_ in zip(values, ids):
        if id_["source"] == "system":
            system_inputs[id_["name"]] = val
        elif id_["source"] == "controller":
            controller_inputs[id_["name"]] = val

    return {"system": system_inputs, "controller": controller_inputs}


# Run simulation
@app.callback(
    Output("simulation-plots-container", "children"),
    Input("run-btn", "n_clicks"),
    State("system-selector", "value"),
    State("system-input-store", "data"),
)
def run_simulation(n_clicks, system_key, inputs):
    if not n_clicks or not inputs:
        return dash.no_update

    system_class = SIM_OPTIONS[system_key]

    # Combine system and controller inputs
    system_inputs = inputs.get("system", {})
    controller_inputs = inputs.get("controller", {})

    system = system_class(**system_inputs, controller_inputs=controller_inputs)
    system.simulate()

    return system.make_simulation_plots()


@app.callback(
    Output("analysis-plots-container", "children"),
    Input("system-input-store", "data"),
    State("system-selector", "value"),
)
def update_analysis_plots(inputs, system_key):
    if not inputs or not system_key:
        return dash.no_update

    system_class = SIM_OPTIONS[system_key]

    # Combine system and controller inputs

    system_inputs = inputs.get("system", {})
    controller_inputs = inputs.get("controller", {})

    system = system_class(**system_inputs, controller_inputs=controller_inputs)

    # Assumes make_analysis_plots() returns Dash components
    return system.make_analysis_fields()


if __name__ == "__main__":
    if LOCAL:
        app.run(debug=True)
    else:
        app.run(debug=False, host="0.0.0.0", port="7860")
