import numpy as np
import scipy as sp
import plotly.graph_objs as go
from dash import Dash, html, dcc, Input, Output, State, MATCH
import dash
from dash.dependencies import ALL
from typing import Type


class PIDController:
    """PID Controller."""

    title = "PID Controller"

    controller_inputs = {
        "Kp": {
            "type": "number",
            "value": 50.0,
            "description": "Proportional gain (Kp)",
        },
        "Ki": {"type": "number", "value": 0.0, "description": "Integral gain (Ki)"},
        "Kd": {"type": "number", "value": 5.0, "description": "Derivative gain (Kd)"},
        "y_target": {
            "type": "number",
            "value": 1.0,
            "description": "Target position (y_target)",
        },
    }

    @classmethod
    def make_layout(cls):
        """TODO"""
        return html.Div(
            [
                html.H2(f"{cls.title} Inputs"),
                *[
                    html.Div(
                        [
                            html.Label(props["description"]),
                            dcc.Input(
                                id={
                                    "type": "controller-input",
                                    "name": name,
                                },
                                type=props["type"],
                                value=props["value"],
                            ),
                        ]
                    )
                    for name, props in cls.controller_inputs.items()
                ],
            ]
        )
    
    # @classmethod
    # def register_callbacks(cls, app):
    #     """TODO"""
    #     # Store controller inputs
    #     @app.callback(
    #         Output({"type": "controller-input-store", "controller": MATCH}, "data"),
    #         [Input({"type": "controller-input", "controller": MATCH, "name": ALL}, "value")],
    #         [State({"type": "controller-input", "controller": MATCH, "name": ALL}, "id")],
    #     )
    #     def store_controller_inputs(values, ids):
    #         return {id_["name"]: val for val, id_ in zip(values, ids)}

    def __init__(self, y_target, Kp, Ki, Kd):
        self.y_target = y_target
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = None

    def initialize(self, dt):
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, output: float) -> float:
        """TODO"""
        error = self.y_target - output

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Update integral state
        self.integral += error * self.dt

        # Compute control input
        return u


class MassSpringSystem:
    """TODO"""

    title = "Mass-Spring System"

    compatible_controllers = {
        "PID Controller": PIDController,
    }

    system_inputs = {
        "mass": {
            "type": "number",
            "value": 1.0,
            "description": "Mass of the system (m)",
        },
        "spring_constant": {
            "type": "number",
            "value": 10.0,
            "description": "Spring constant (k)",
        },
        "damping_coefficient": {
            "type": "number",
            "value": 0.0,
            "description": "Damping coefficient (c)",
        },
    }

    simulation_inputs = {
        "final_time": {
            "type": "number",
            "value": 10.0,
            "description": "Simulation time (s)",
        },
        "dt": {"type": "number", "value": 0.01, "description": "Time step (dt)"},
    }

    @classmethod
    def make_layout(cls):
        """TODO"""

        def make_input_fields(inputs: dict) -> list:
            """TODO"""
            input_fields = []
            for name, props in inputs.items():
                input_fields.extend(
                    [
                        html.Label(props["description"]),
                        dcc.Input(
                            id={"type": "system-input", "name": name},
                            type=props["type"],
                            value=props["value"],
                        ),
                    ]
                )
            return input_fields

        return html.Div(
            [
                html.H1(cls.title),
                # System Inputs
                html.H2("System Inputs"),
                html.Div(make_input_fields(cls.system_inputs)),
                # # Controller Options
                # html.H2("Controller Options"),
                # dcc.Dropdown(
                #     id={"type": "controller-selection"},
                #     options=[{"label": controller_key, "value": controller_key} for controller_key in cls.compatible_controllers],
                #     value="PID Controller",
                # ),
                # # Controller Inputs
                # html.Div(
                #     id={"type": "controller-container"},
                # ),
                # Simulation Inputs
                html.H2("Simulation Inputs"),
                html.Div(make_input_fields(cls.simulation_inputs)),
                # # Run Button
                # html.Button(
                #     "Run Simulation", id={"type": "run-btn"}, n_clicks=0
                # ),
                # # Plots Container
                # html.Div(id={"type": "plots-container"}),
            ]
        )

    # @classmethod
    # def register_callbacks(cls, app):
        # Store system inputs
        # @app.callback(
        #     Output({"type": "system-input-store", "sim": MATCH}, "data"),
        #     [Input({"type": "sim-input", "sim": MATCH, "name": ALL}, "value")],
        #     [State({"type": "sim-input", "sim": MATCH, "name": ALL}, "id")],
        # )
        # def store_system_inputs(values, ids):
        #     # TODO change the pattern matching of this
        #     return {id_["name"]: val for val, id_ in zip(values, ids)}

        # Dynamically generate State inputs from class dictionaries
        # state_inputs = [
        #     State({"type": "sim-input", "sim": MATCH, "name": name}, "value")
        #     for name in list(cls.system_inputs.keys())
        #     + list(cls.simulation_inputs.keys())
        # ]

        # TODO now im going to need to seperate out the callbacks for running the simulation and controlling the controller and everything else
        # @app.callback(
        #     Output({"type": "controller-container", "sim": MATCH}, "children"),
        #     Input({"type": "controller-selection", "sim": MATCH}, "value"),
        # )
        # def update_controller_container(controller_key):
        #     controller_class = cls.compatible_controllers[controller_key]
        #     return controller_class.make_layout(controller_key)

        # @app.callback(
        #     Output({"type": "plots-container", "sim": MATCH}, "children"),
        #     Input({"type": "run-btn", "sim": MATCH}, "n_clicks"),
        #     State({"type": "system-input-store", "sim": MATCH}, "data"),
        #     State({"type": "controller-input-store", "controller": MATCH}, "data"),
        # )
        # def run_simulation(n_clicks, system_inputs, controller_inputs):
        #     if n_clicks == 0:
        #         return dash.no_update
        #     # Example: unpack system and controller inputs
        #     # You may need to adjust argument names to match your class constructors
        #     system = MassSpringSystem(
        #         mass=system_inputs["mass"],
        #         spring_constant=system_inputs["spring_k"],
        #         damping_coefficient=system_inputs["damping_coefficient"],
        #         state=np.array([0.0, 0.0], dtype=float),
        #         **controller_inputs
        #     )
        #     system.simulate(system_inputs["final_time"], system_inputs["dt"])
        #     return system.make_plots()

    def __init__(
        self,
        mass: float,
        spring_constant: float,
        damping_coefficient: float,
        # Simulation inputs
        final_time: float,
        dt: float,
        # state: np.ndarray = np.array([0.0, 0.0], dtype=float), # TODO this needs to be changed
        # going to want to abstract the controller logic later
        controller: PIDController
    ):
        """TODO state is a 2D vector [position, velocity]"""
        self.mass = mass
        self.spring_constant = spring_constant
        self.damping_coefficient = damping_coefficient
        self.state = np.array([0.0, 0.0], dtype=float)

        self.final_time = final_time
        self.dt = dt

        # TODO abstract the controller logic
        self.controller = controller

    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        The state space model of the mass-spring system
        Defines x dot given x, u
        Change later if we want non time variant
        """
        return np.dot(self.A(), x) + np.dot(self.B(), u)

    def g(self, x: np.ndarray) -> np.ndarray:
        return np.dot(self.C(), x)

    def A(self) -> np.ndarray:
        """TODO"""
        return np.array(
            [
                [0, 1],
                [
                    -self.spring_constant / self.mass,
                    -self.damping_coefficient / self.mass,
                ],
            ]
        )

    def B(self) -> np.ndarray:
        """TODO"""
        return np.array([[0], [1 / self.mass]])

    def C(self) -> np.ndarray:
        return np.array([[1, 0]])

    def simulate(self) -> None:
        """
        TODO
        Simulate the system for one time step given control input u
        Update the state in place
        """
        # Initialize the states of the system
        self.t = np.arange(0, self.final_time, self.dt)
        self.x = np.zeros((len(self.t), len(self.state)))
        self.x[0] = self.state
        self.u = np.zeros((len(self.t), 1))
        self.y = np.zeros(len(self.t))
        self.y[0] = self.g(self.x[0])

        self.controller.initialize(self.dt)

        # Forward Euler integration TODO this can probably be abstracted or done in way that allows other integrators to be selected
        for i in range(0, len(self.t) - 1):
            self.u[i] = self.controller.step(self.y[i])

            # Update state
            self.x[i + 1] = self.x[i] + self.f(self.x[i], self.u[i]) * self.dt
            self.y[i + 1] = self.g(self.x[i + 1])

        self.state = self.x[-1]  # Update the state to the last computed state

    def make_plots(self):
        """
        Create a Div containing all plots for the mass-spring system.
        """
        figures = [
            self.make_animation(),
            self.position_plot(),
            # Add more plot methods here if needed
        ]
        return html.Div([dcc.Graph(figure=fig) for fig in figures])

    def make_animation(self) -> go.Figure:
        """
        Create an animation of the mass-spring system
        """
        # Axis limits
        y_min = np.min(self.x[:, 0]) - 0.5
        y_max = np.max(self.x[:, 0]) + 0.5

        # Initial trace
        spring = go.Scatter(
            x=[0, self.x[0, 0]],
            y=[0, 0],
            mode="lines+markers",
            marker=dict(size=[1, 20]),
            line=dict(width=4),
        )

        # Frames for animation
        frames = [
            go.Frame(
                data=[
                    go.Scatter(
                        x=[0, self.x[i, 0]],
                        y=[0, 0],
                        mode="lines+markers",
                        marker=dict(size=[1, 20]),
                        line=dict(width=4),
                    )
                ],
                name=str(i),
                layout=go.Layout(
                    annotations=[
                        dict(
                            x=0.5,
                            y=1.05,
                            xref="paper",
                            yref="paper",
                            text=f"Sim Time: {self.t[i]:.2f}s",
                            showarrow=False,
                            font=dict(size=16),
                        )
                    ]
                ),
            )
            for i in range(len(self.t))
        ]

        fig = go.Figure(
            data=[spring],
            layout=go.Layout(
                title="Mass-Spring Animation",
                xaxis=dict(range=[y_min, y_max]),
                yaxis=dict(range=[-1, 1]),
                showlegend=False,
                updatemenus=[
                    {
                        "buttons": [
                            {
                                "args": [
                                    None,
                                    {
                                        "frame": {"duration": 100, "redraw": True},
                                        "fromcurrent": True,
                                    },
                                ],
                                "label": "Play",
                                "method": "animate",
                            },
                            {
                                "args": [
                                    [None],
                                    {
                                        "frame": {"duration": 0, "redraw": True},
                                        "mode": "immediate",
                                        "transition": {"duration": 0},
                                    },
                                ],
                                "label": "Pause",
                                "method": "animate",
                            },
                        ],
                        "direction": "left",
                        "pad": {"r": 10, "t": 87},
                        "showactive": False,
                        "type": "buttons",
                        "x": 0.1,
                        "xanchor": "right",
                        "y": 0,
                        "yanchor": "top",
                    }
                ],
                sliders=[
                    {
                        "steps": [
                            {
                                "args": [
                                    [str(i)],
                                    {
                                        "frame": {
                                            "duration": int(100 / 1.0),
                                            "redraw": True,
                                        },
                                        "mode": "immediate",
                                    },
                                ],
                                "label": f"{self.t[i]:.2f}s",
                                "method": "animate",
                            }
                            for i in range(len(self.t))
                        ],
                        "active": 0,
                        "yanchor": "top",
                        "xanchor": "left",
                        "currentvalue": {"prefix": "Sim Time: ", "font": {"size": 16}},
                        "transition": {"duration": 0, "easing": "cubic-in-out"},
                    }
                ],
            ),
            frames=frames,
        )
        return fig

    def position_plot(self) -> go.Figure:
        """TODO"""

        # Determine axis limits based on max/min position
        y_min = np.min(np.min(self.x[:, 0])) - 0.5
        y_max = np.max(np.max(self.x[:, 0])) + 0.5

        # Position vs Time
        position_plot = go.Figure()
        position_plot.add_trace(go.Scatter(x=self.t, y=self.y, mode="lines"))
        position_plot.update_layout(
            title="Mass Position Over Time",
            xaxis_title="Time (s)",
            yaxis_title="Position (m)",
            yaxis=dict(range=[y_min, y_max]),
        )
        return position_plot


SIM_OPTIONS: dict[str, type[MassSpringSystem]] = {
    "Mass-Spring System": MassSpringSystem,
}

CONTROLLER_OPTIONS: dict[str, type[PIDController]] = {
    "PID Controller": PIDController,
}

app = Dash(__name__)

app.layout = html.Div([
    dcc.Dropdown(
        id="system-selector",
        options=[{"label": k, "value": k} for k in SIM_OPTIONS],
        value=list(SIM_OPTIONS.keys())[0],
    ),
    dcc.Dropdown(
        id="controller-selector",
        options=[{"label": k, "value": k} for k in CONTROLLER_OPTIONS],
        value=list(CONTROLLER_OPTIONS.keys())[0],
    ),
    html.Div(id="system-inputs-container"),
    html.Div(id="controller-inputs-container"),
    html.Button("Run Simulation", id="run-btn"),
    html.Div(id="plots-container"),
    dcc.Store(id="system-input-store"),
    dcc.Store(id="controller-input-store"),
])

# Dynamically generate system input fields
@app.callback(
    Output("system-inputs-container", "children"),
    Input("system-selector", "value"),
)
def render_system_inputs(system_key):
    system_class = SIM_OPTIONS[system_key]
    return system_class.make_layout()

# Dynamically generate controller input fields TODO move this to the system class
@app.callback(
    Output("controller-inputs-container", "children"),
    Input("controller-selector", "value"),
)
def render_controller_inputs(controller_key):
    controller_class: type[PIDController] = CONTROLLER_OPTIONS[controller_key]
    return controller_class.make_layout()

# Store system inputs
@app.callback(
    Output("system-input-store", "data"),
    Input({"type": "system-input", "name": ALL}, "value"),
    State({"type": "system-input", "name": ALL}, "id"),
)
def store_system_inputs(values, ids):
    print("Storing system inputs:", values, ids)
    return {id_["name"]: val for val, id_ in zip(values, ids)}

# Store controller inputs
@app.callback(
    Output("controller-input-store", "data"),
    Input({"type": "controller-input", "name": ALL}, "value"),
    State({"type": "controller-input", "name": ALL}, "id"),
)
def store_controller_inputs(values, ids):
    print("Storing controller inputs:", values, ids)
    return {id_["name"]: val for val, id_ in zip(values, ids)}

# Run simulation
@app.callback(
    Output("plots-container", "children"),
    Input("run-btn", "n_clicks"),
    State("system-selector", "value"),
    State("controller-selector", "value"),
    State("system-input-store", "data"),
    State("controller-input-store", "data"),
)
def run_simulation(n_clicks, system_key, controller_key, system_inputs, controller_inputs):
    if not n_clicks:
        return dash.no_update
    print("Running simulation with inputs:", system_inputs, controller_inputs)
    system_class = SIM_OPTIONS[system_key]
    controller_class = CONTROLLER_OPTIONS[controller_key]
    # Unpack and pass arguments as needed
    controller = controller_class(**controller_inputs)
    system = system_class(**system_inputs, controller=controller)
    system.simulate()
    return system.make_plots()

# for sim_class in SIM_OPTIONS.values():
#     sim_class.register_callbacks(app)

# for controller_class in CONTROLLER_OPTIONS.values():
#     controller_class.register_callbacks(app)

if __name__ == "__main__":
    app.run(debug=True)
