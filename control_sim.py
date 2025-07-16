import numpy as np
import scipy as sp
import plotly.graph_objs as go
from dash import Dash, html, dcc, Input, Output, State, MATCH
import dash


class MassSpringSystem:
    """TODO"""

    @staticmethod
    def make_layout(sim_key):
        print("making layout")
        return html.Div(
            [
                html.Label("Mass (m):"),
                dcc.Input(
                    id={"type": "sim-input", "sim": sim_key, "name": "mass"},
                    type="number",
                    value=1.0,
                ),
                html.Label("Spring Constant (k):"),
                dcc.Input(
                    id={"type": "sim-input", "sim": sim_key, "name": "spring_k"},
                    type="number",
                    value=10.0,
                ),
                html.Label("Target Position (y_target):"),
                dcc.Input(
                    id={"type": "sim-input", "sim": sim_key, "name": "y_target"},
                    type="number",
                    value=1.0,
                ),
                html.Label("PID: Kp:"),
                dcc.Input(
                    id={"type": "sim-input", "sim": sim_key, "name": "Kp"},
                    type="number",
                    value=50.0,
                ),
                html.Label("Ki:"),
                dcc.Input(
                    id={"type": "sim-input", "sim": sim_key, "name": "Ki"},
                    type="number",
                    value=0.0,
                ),
                html.Label("Kd:"),
                dcc.Input(
                    id={"type": "sim-input", "sim": sim_key, "name": "Kd"},
                    type="number",
                    value=5.0,
                ),
                html.Label("Simulation Time (s):"),
                dcc.Input(
                    id={"type": "sim-input", "sim": sim_key, "name": "sim_time"},
                    type="number",
                    value=10.0,
                ),
                html.Label("Time Step (Dt):"),
                dcc.Input(
                    id={"type": "sim-input", "sim": sim_key, "name": "Dt"},
                    type="number",
                    value=0.01,
                ),
                html.Button(
                    "Run Simulation", id={"type": "run-btn", "sim": sim_key}, n_clicks=0
                ),
                dcc.Graph(id={"type": "spring_animation", "sim": sim_key}),
                dcc.Graph(id={"type": "position_plot", "sim": sim_key}),
            ],
            style={"columnCount": 2, "gap": "20px"},
        )

    @staticmethod
    def register_callback(app):
        print("Registering callbacks for MassSpringSystem")

        @app.callback(
            Output({"type": "spring_animation", "sim": MATCH}, "figure"),
            Output({"type": "position_plot", "sim": MATCH}, "figure"),
            Input({"type": "run-btn", "sim": MATCH}, "n_clicks"),
            State({"type": "sim-input", "sim": MATCH, "name": "mass"}, "value"),
            State({"type": "sim-input", "sim": MATCH, "name": "spring_k"}, "value"),
            State({"type": "sim-input", "sim": MATCH, "name": "y_target"}, "value"),
            State({"type": "sim-input", "sim": MATCH, "name": "Kp"}, "value"),
            State({"type": "sim-input", "sim": MATCH, "name": "Ki"}, "value"),
            State({"type": "sim-input", "sim": MATCH, "name": "Kd"}, "value"),
            State({"type": "sim-input", "sim": MATCH, "name": "sim_time"}, "value"),
            State({"type": "sim-input", "sim": MATCH, "name": "Dt"}, "value"),
        )
        def run_simulation(n_clicks, m, k, y_target, Kp, Ki, Kd, sim_time, dt):
            if n_clicks == 0:
                return dash.no_update, dash.no_update
            system = MassSpringSystem(
                m, k, 0.0, np.array([0.0, 0.0], dtype=float), y_target, Kp, Ki, Kd
            )
            system.simulate(sim_time, dt)
            animation = system.make_animation()
            position_plot = system.make_plots()
            return animation, position_plot

    def __init__(
        self,
        mass: float,
        spring_constant: float,
        damping_coefficient: float,
        state: np.ndarray,
        y_target: float,
        # going to want to abstract the controller logic later
        Kp: float,
        Ki: float,
        Kd: float,
    ):
        """TODO state is a 2D vector [position, velocity]"""
        self.mass = mass
        self.spring_constant = spring_constant
        self.damping_coefficient = damping_coefficient
        self.state = state

        # TODO abstract the controller logic
        self.y_target = y_target
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

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

    def simulate(self, tf: float, dt: float) -> None:
        """
        TODO
        Simulate the system for one time step given control input u
        Update the state in place
        """
        # Initialize the states of the system
        self.t = np.arange(0, tf, dt)
        self.x = np.zeros((len(self.t), len(self.state)))
        self.x[0] = self.state
        self.u = np.zeros((len(self.t), 1))
        self.y = np.zeros(len(self.t))
        self.y[0] = self.g(self.x[0])

        # Initialize the controller states
        integral = 0.0
        prev_error = 0.0

        # Forward Euler integration TODO this can probably be abstracted or done in way that allows other integrators to be selected
        for i in range(0, len(self.t) - 1):
            # Compute control input
            error = self.y_target - self.y[i]
            derivative = (error - prev_error) / dt

            self.u[i] = self.Kp * error + self.Ki * integral + self.Kd * derivative
            prev_error = error

            integral = integral + error * dt

            # Update state
            self.x[i + 1] = self.x[i] + self.f(self.x[i], self.u[i]) * dt
            self.y[i + 1] = self.g(self.x[i + 1])

        self.state = self.x[-1]  # Update the state to the last computed state

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

    def make_plots(self) -> go.Figure:
        """
        Create plots for the mass-spring system
        """

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


SIM_OPTIONS = {
    "Mass-Spring System": MassSpringSystem,
}

app = Dash(__name__)

app.layout = html.Div(
    [
        html.H1("PID-Controlled Mass-Spring System"),
        dcc.Dropdown(
            id="sim-selector",
            options=[{"label": k, "value": k} for k in SIM_OPTIONS],
            value="Mass-Spring System",
        ),
        html.Div(id="sim-container"),
    ]
)


@app.callback(Output("sim-container", "children"), Input("sim-selector", "value"))
def update_sim_layout(sim_key):
    sim_class = SIM_OPTIONS[sim_key]
    return sim_class.make_layout(sim_key)


for sim_class in SIM_OPTIONS.values():
    sim_class.register_callback(app)

if __name__ == "__main__":
    app.run(debug=True)
