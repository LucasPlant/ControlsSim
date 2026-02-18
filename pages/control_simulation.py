import dash
from dash import Input, Output, State, callback, dcc, html, register_page
from dash.dependencies import ALL

from systems import BaseSystem, MassSpringSystem, MotorizedPendulum, TurtleBot
from utils import make_input_field

register_page(__name__, path="/control-simulation", name="Control Simulation")

SIM_OPTIONS: dict[str, type[BaseSystem]] = {
    "Mass-Spring System": MassSpringSystem,
    "TurtleBot": TurtleBot,
    "Motorized Pendulum": MotorizedPendulum,
}

layout = html.Main(
    [
        html.Div(
            [
                dcc.Link("Back to Home", href="/", className="back-link"),
                html.H1("Control Simulation", className="page-title"),
            ],
            className="sim-header",
        ),
        html.Div(
            [
                html.Div(
                    [
                        html.H2("Select a System", className="section-title"),
                        dcc.Dropdown(
                            id="system-selector",
                            options=[{"label": k, "value": k} for k in SIM_OPTIONS],
                            value=list(SIM_OPTIONS.keys())[0],
                            clearable=False,
                            className="dropdown",
                        ),
                        html.Div(id="system-inputs-container"),
                        html.Button("Run Simulation", id="run-btn", className="primary-btn"),
                    ],
                    className="panel sim-column",
                ),
                html.Div(
                    [
                        html.Div(id="analysis-plots-container", className="panel"),
                        html.Div(id="simulation-plots-container", className="results-wrap"),
                    ],
                    className="sim-column",
                ),
            ],
            className="sim-split",
        ),
        dcc.Store(id="system-input-store"),
    ],
    className="simulation-page",
)


@callback(
    Output("system-inputs-container", "children"),
    Input("system-selector", "value"),
)
def render_system_args(system_key):
    system_class = SIM_OPTIONS[system_key]
    return system_class.make_layout({}, {})


@callback(
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
    input_info = system_class.input_info
    output_info = system_class.output_info
    state_info = system_class.state_info

    if not controller_class:
        return [html.Div()]

    controller_inputs = {}
    if inputs and "controller" in inputs:
        controller_inputs = inputs["controller"]

    return [
        html.Div(
            [
                controller_class.make_layout(
                    controller_inputs, input_info, output_info, state_info
                ),
            ]
        )
    ]


@callback(
    Output({"type": "trajectory-inputs", "controller": ALL}, "children"),
    Input(
        {"type": "input", "source": "controller", "name": "trajectory_generator"},
        "value",
    ),
    State({"type": "input", "source": "system", "name": "controller_type"}, "value"),
    State("system-selector", "value"),
    State("system-input-store", "data"),
)
def render_trajectory_inputs(trajectory_generator_type, controller_type, system_key, inputs):
    if not trajectory_generator_type or not controller_type or not system_key:
        return [html.Div()]

    system_class = SIM_OPTIONS[system_key]
    controller_class = system_class.allowed_controllers.get(controller_type)

    if not controller_class:
        return [html.Div()]

    input_info = system_class.input_info
    output_info = system_class.output_info

    trajectory_generator = controller_class.trajectory_generators.get(
        trajectory_generator_type
    )
    if not trajectory_generator:
        return [html.Div()]

    trajectory_inputs = {}
    if (
        inputs
        and "controller" in inputs
        and "trajectory_generator_inputs" in inputs["controller"]
    ):
        trajectory_inputs = inputs["controller"]["trajectory_generator_inputs"]

    trajectory_inputs_fields = [
        make_input_field(name, props, "trajectory_generator", trajectory_inputs)
        for name, props in trajectory_generator.get_inputs(input_info, output_info).items()
    ]

    return [
        html.Div(
            [
                html.H4("Trajectory Generation Parameters"),
                *trajectory_inputs_fields,
            ]
        )
    ]


@callback(
    Output("system-input-store", "data"),
    Input({"type": "input", "source": ALL, "name": ALL}, "value"),
    State({"type": "input", "source": ALL, "name": ALL}, "id"),
)
def store_inputs(values, ids):
    system_args = {}
    controller_inputs = {}
    trajectory_generator_inputs = {}
    initial_controller_state = {}

    for val, id_ in zip(values, ids):
        if id_["source"] == "system":
            system_args[id_["name"]] = val
        elif id_["source"] == "controller":
            controller_inputs[id_["name"]] = val
        elif id_["source"] == "trajectory_generator":
            trajectory_generator_inputs[id_["name"]] = val
        elif id_["source"] == "controller-state":
            initial_controller_state[id_["name"]] = val

    controller_inputs["trajectory_generator_inputs"] = trajectory_generator_inputs
    controller_inputs["initial_state"] = initial_controller_state

    return {"system": system_args, "controller": controller_inputs}


@callback(
    Output("simulation-plots-container", "children"),
    Input("run-btn", "n_clicks"),
    State("system-selector", "value"),
    State("system-input-store", "data"),
)
def run_simulation(n_clicks, system_key, inputs):
    if not n_clicks or not inputs:
        return dash.no_update

    system_class = SIM_OPTIONS[system_key]
    system_args = inputs.get("system", {})
    controller_inputs = inputs.get("controller", {})

    system = system_class(**system_args, controller_inputs=controller_inputs)
    system.simulate()

    return system.make_simulation_plots()


@callback(
    Output("analysis-plots-container", "children"),
    Input("system-input-store", "data"),
    State("system-selector", "value"),
)
def update_analysis_plots(inputs, system_key):
    if not inputs or not system_key:
        return dash.no_update

    system_class = SIM_OPTIONS[system_key]
    system_args = inputs.get("system", {})
    controller_inputs = inputs.get("controller", {})

    system = system_class(**system_args, controller_inputs=controller_inputs)
    return system.make_analysis_fields()
