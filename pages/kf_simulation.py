import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "2d_kf_sim"))

_KF_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "2d_kf_sim")
_EKF_MATH_MD = open(os.path.join(_KF_DIR, "kalman_filter_math.md")).read()
# _SIM_README_MD = open(os.path.join(_KF_DIR, "sim_planning.md")).read()

import numpy as np
import dash
from dash import Input, Output, State, callback, dcc, html, register_page
from dash.dependencies import ALL

from sim import NoiseConfig, CircleCenterFacing, CircleTangentFacing, SinusoidForward, RandomWalk
from EKF import EKF
from kf_plot_utils import (
    plot_trajectory,
    animate_trajectory,
    plot_imu_measurements,
    plot_trajectory_with_bounds,
    animate_estimate,
    plot_ekf_states,
    plot_mc_estimates,
    plot_mc_mse,
)

register_page(__name__, path="/kf-simulation", name="SE2 Kalman Filter")


# ---------------------------------------------------------------------------
# Defaults and limits
# ---------------------------------------------------------------------------

# General simulation parameters
DEFAULT_DT       = 0.02
DEFAULT_T_FINAL  = 10.0
DEFAULT_N_TRIALS = 10
DEFAULT_MASS     = 1.0
DEFAULT_INERTIA  = 0.5
DEFAULT_SIGMA0   = 5.0      # initial velocity std dev [m/s]

MIN_DT,       MAX_DT       = 0.001, 0.5
MIN_T_FINAL,  MAX_T_FINAL  = 1.0,   300.0
MIN_N_TRIALS, MAX_N_TRIALS = 1,     100
MIN_MASS,     MAX_MASS     = 0.01,  50.0
MIN_INERTIA,  MAX_INERTIA  = 0.01,  50.0
MIN_SIGMA0,   MAX_SIGMA0   = 0.1,   50.0

# Noise configuration (all std devs — squared in callback before use)
DEFAULT_IMU_ACCEL_STD_X = 0.15  # [m/s²]
DEFAULT_IMU_ACCEL_STD_Y = 0.15  # [m/s²]
DEFAULT_IMU_GYRO_STD    = 0.1   # [rad/s]
DEFAULT_POS_STD_X       = 0.3   # [m]
DEFAULT_POS_STD_Y       = 0.3   # [m]
DEFAULT_HEADING_STD     = 0.2   # [rad]

MIN_IMU_ACCEL_STD, MAX_IMU_ACCEL_STD = 0.0, 10.0
MIN_IMU_GYRO_STD,  MAX_IMU_GYRO_STD  = 0.0,  5.0
MIN_POS_STD,       MAX_POS_STD       = 0.0, 10.0
MIN_HEADING_STD,   MAX_HEADING_STD   = 0.0,  3.15

# Initial state limits [x/y in m, theta in deg, v in m/s, omega in rad/s]
MIN_POS,   MAX_POS   = -200.0, 200.0
MIN_THETA, MAX_THETA = -180.0, 180.0
MIN_VEL,   MAX_VEL   = -50.0,  50.0
MIN_OMEGA, MAX_OMEGA = -20.0,  20.0

# Step sizes for numeric inputs
STEP_DT            = 0.001
STEP_T_FINAL       = 1.0
STEP_N_TRIALS      = 1
STEP_MASS          = 0.01
STEP_INERTIA       = 0.01
STEP_SIGMA0        = 0.1
STEP_IMU_ACCEL_STD = 0.01
STEP_IMU_GYRO_STD  = 0.01
STEP_POS_STD       = 0.01
STEP_HEADING_STD   = 0.01
STEP_POS           = 0.1
STEP_THETA         = 1.0
STEP_VEL           = 0.1
STEP_OMEGA         = 0.1
STEP_FREQ          = 0.01
STEP_GAIN          = 0.5


# ---------------------------------------------------------------------------
# Registries
# ---------------------------------------------------------------------------

SIM_OPTIONS = {
    "circle_tangent": "Circle — Tangent Facing",
    "circle_center":  "Circle — Center Facing",
    "sinusoid":       "Forward Sinusoid",
    "random_walk":    "Random Walk",
}

SIM_CLASSES = {
    "circle_tangent": CircleTangentFacing,
    "circle_center":  CircleCenterFacing,
    "sinusoid":       SinusoidForward,
    "random_walk":    RandomWalk,
}

# (name, label, default, min, max, step)
SIM_SPECIFIC_FIELDS = {
    "circle_tangent": [
        ("radius", "Radius (m)",  5.0, 0.1,  50.0, STEP_POS),
        ("speed",  "Speed (m/s)", 2.0, 0.1,  20.0, STEP_VEL),
    ],
    "circle_center": [
        ("radius", "Radius (m)",  5.0, 0.1,  50.0, STEP_POS),
        ("speed",  "Speed (m/s)", 2.0, 0.1,  20.0, STEP_VEL),
    ],
    "sinusoid": [
        ("speed",    "Speed (m/s)",          2.0,  0.1,  20.0, STEP_VEL),
        ("lat_amp",  "Lat Amplitude (m/s)",  1.0,  0.0,  10.0, STEP_VEL),
        ("lat_freq", "Lat Frequency (Hz)",   0.3,  0.01,  5.0, STEP_FREQ),
        ("kv",       "Velocity Gain",        8.0,  0.1,  50.0, STEP_GAIN),
    ],
    "random_walk": [
        ("force_std",  "Force Std (N)",    1.5, 0.0, 20.0, STEP_VEL),
        ("torque_std", "Torque Std (N·m)", 1.0, 0.0, 20.0, STEP_VEL),
    ],
}

# [x0, y0, theta0_deg, vx0, vy0, omega0]
DEFAULT_INIT = {
    "circle_tangent": [5.0, 0.0,  90.0, 0.0, 2.0, 0.0],
    "circle_center":  [5.0, 0.0, 180.0, 0.0, 2.0, 0.0],
    "sinusoid":       [0.0, 0.0,  90.0, 0.0, 2.0, 0.0],
    "random_walk":    [0.0, 0.0,   0.0, 0.0, 0.0, 0.0],
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _labeled_input(label, id_, value, min_=None, max_=None, step=None):
    return html.Div(
        [
            html.Label(label),
            dcc.Input(
                id=id_,
                type="number",
                value=value,
                min=min_,
                max=max_,
                step=step,
                debounce=True,
            ),
        ]
    )


def _build_ekf_inputs(traj):
    z = np.concatenate([traj.pos_meas_W, traj.heading_meas_W[:, :, None]], axis=-1)
    u = np.concatenate([traj.accel_meas_B, traj.gyro_meas_B[:, :, None]], axis=-1)
    return z, u


def _collect_sim_kw(values, ids):
    """Extract sim-specific kwargs from pattern-matching callback values."""
    return {id_["name"]: float(v) for v, id_ in zip(values, ids) if v is not None}


def _details(summary, children, open_=False):
    return html.Details(
        [html.Summary(summary, className="menu-summary")] + children,
        open=open_,
        className="menu-group",
    )


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------

layout = html.Main(
    [
        html.Div(
            [
                dcc.Link("Back to Home", href="/", className="back-link"),
                html.H1("SE2 Kalman Filter", className="page-title"),
            ],
            className="sim-header",
        ),
        html.Div(
            [
                # ---- LEFT COLUMN: inputs -----
                html.Div(
                    [
                        html.H2("Select a Simulation", className="section-title"),
                        dcc.Dropdown(
                            id="kf-sim-type",
                            options=[{"label": v, "value": k} for k, v in SIM_OPTIONS.items()],
                            value="sinusoid",
                            clearable=False,
                            className="dropdown",
                        ),
                        _details(
                            "Simulation Inputs",
                            [html.Div(id="kf-sim-specific-inputs")],
                            open_=False,
                        ),
                        _details(
                            "General Parameters",
                            [
                                html.Div([
                                    _labeled_input("DT (s)",       "kf-dt",       DEFAULT_DT,       MIN_DT,       MAX_DT,       STEP_DT),
                                    _labeled_input("T Final (s)",  "kf-t-final",  DEFAULT_T_FINAL,  MIN_T_FINAL,  MAX_T_FINAL,  STEP_T_FINAL),
                                    _labeled_input("N Trials",     "kf-n-trials", DEFAULT_N_TRIALS, MIN_N_TRIALS, MAX_N_TRIALS, STEP_N_TRIALS),
                                    _labeled_input("Mass (kg)",    "kf-mass",     DEFAULT_MASS,     MIN_MASS,     MAX_MASS,     STEP_MASS),
                                    _labeled_input("Inertia",      "kf-inertia",  DEFAULT_INERTIA,  MIN_INERTIA,  MAX_INERTIA,  STEP_INERTIA),
                                    _labeled_input("Sigma0 (m/s)", "kf-sigma0",   DEFAULT_SIGMA0,   MIN_SIGMA0,   MAX_SIGMA0,   STEP_SIGMA0),
                                ]),
                            ],
                        ),
                        _details(
                            "Initial State",
                            [html.Div(id="kf-init-state-inputs")],
                        ),
                        _details(
                            "Noise Configuration",
                            [
                                html.Div([
                                    _labeled_input("IMU Accel Std X (m/s²)", "kf-imu-ax-std", DEFAULT_IMU_ACCEL_STD_X, MIN_IMU_ACCEL_STD, MAX_IMU_ACCEL_STD, STEP_IMU_ACCEL_STD),
                                    _labeled_input("IMU Accel Std Y (m/s²)", "kf-imu-ay-std", DEFAULT_IMU_ACCEL_STD_Y, MIN_IMU_ACCEL_STD, MAX_IMU_ACCEL_STD, STEP_IMU_ACCEL_STD),
                                    _labeled_input("IMU Gyro Std (rad/s)",   "kf-gyro-std",   DEFAULT_IMU_GYRO_STD,    MIN_IMU_GYRO_STD,  MAX_IMU_GYRO_STD,  STEP_IMU_GYRO_STD),
                                    _labeled_input("Pos Std X (m)",          "kf-pos-sx",     DEFAULT_POS_STD_X,       MIN_POS_STD,       MAX_POS_STD,       STEP_POS_STD),
                                    _labeled_input("Pos Std Y (m)",          "kf-pos-sy",     DEFAULT_POS_STD_Y,       MIN_POS_STD,       MAX_POS_STD,       STEP_POS_STD),
                                    _labeled_input("Heading Std (rad)",      "kf-hdg-std",    DEFAULT_HEADING_STD,     MIN_HEADING_STD,   MAX_HEADING_STD,   STEP_HEADING_STD),
                                ]),
                            ],
                        ),
                        html.Button(
                            "Run Simulation",
                            id="kf-run-btn",
                            className="primary-btn",
                            style={"width": "100%", "marginTop": "16px"},
                        ),
                    ],
                    className="panel sim-column",
                ),
                # ---- RIGHT COLUMN: plots ----
                html.Div(
                    [
                        dcc.Loading(
                            html.Div(id="kf-plots-container"),
                            type="circle",
                        ),
                    ],
                    className="sim-column",
                ),
            ],
            className="sim-split",
        ),
        html.Div(
            [
                dcc.Markdown(_EKF_MATH_MD, mathjax=True, className="math-doc"),
                # dcc.Markdown(_SIM_README_MD, mathjax=True, className="math-doc"),
            ],
            className="math-doc-section",
        ),
    ],
    className="simulation-page",
)


# ---------------------------------------------------------------------------
# Callbacks
# ---------------------------------------------------------------------------

@callback(
    Output("kf-sim-specific-inputs", "children"),
    Input("kf-sim-type", "value"),
)
def render_sim_inputs(sim_type):
    return [
        _labeled_input(
            label,
            {"type": "kf-sim-specific", "name": name},
            default, min_, max_, step,
        )
        for name, label, default, min_, max_, step in SIM_SPECIFIC_FIELDS[sim_type]
    ]


@callback(
    Output("kf-init-state-inputs", "children"),
    Input("kf-sim-type", "value"),
)
def render_init_state(sim_type):
    x0, y0, th0, vx0, vy0, om0 = DEFAULT_INIT[sim_type]
    return [
        _labeled_input("x0 (m)",         "kf-x0",     x0,  MIN_POS,   MAX_POS,   STEP_POS),
        _labeled_input("y0 (m)",         "kf-y0",     y0,  MIN_POS,   MAX_POS,   STEP_POS),
        _labeled_input("theta0 (deg)",   "kf-theta0", th0, MIN_THETA, MAX_THETA, STEP_THETA),
        _labeled_input("vx0 (m/s)",      "kf-vx0",    vx0, MIN_VEL,   MAX_VEL,   STEP_VEL),
        _labeled_input("vy0 (m/s)",      "kf-vy0",    vy0, MIN_VEL,   MAX_VEL,   STEP_VEL),
        _labeled_input("omega0 (rad/s)", "kf-omega0", om0, MIN_OMEGA, MAX_OMEGA, STEP_OMEGA),
    ]


@callback(
    Output("kf-plots-container", "children"),
    Input("kf-run-btn", "n_clicks"),
    State("kf-sim-type",   "value"),
    State("kf-dt",         "value"),
    State("kf-t-final",    "value"),
    State("kf-n-trials",   "value"),
    State("kf-mass",       "value"),
    State("kf-inertia",    "value"),
    State("kf-sigma0",     "value"),
    State("kf-x0",         "value"),
    State("kf-y0",         "value"),
    State("kf-theta0",     "value"),
    State("kf-vx0",        "value"),
    State("kf-vy0",        "value"),
    State("kf-omega0",     "value"),
    State("kf-imu-ax-std", "value"),
    State("kf-imu-ay-std", "value"),
    State("kf-gyro-std",   "value"),
    State("kf-pos-sx",     "value"),
    State("kf-pos-sy",     "value"),
    State("kf-hdg-std",    "value"),
    State({"type": "kf-sim-specific", "name": ALL}, "value"),
    State({"type": "kf-sim-specific", "name": ALL}, "id"),
    prevent_initial_call=True,
)
def run_kf_simulation(
    n_clicks,
    sim_type, dt, t_final, n_trials, mass, inertia, sigma0,
    x0, y0, theta0_deg, vx0, vy0, omega0,
    imu_ax_std, imu_ay_std, gyro_std, pos_sx, pos_sy, hdg_std,
    sim_specific_values, sim_specific_ids,
):
    if not n_clicks:
        return dash.no_update

    sim_kw = _collect_sim_kw(sim_specific_values, sim_specific_ids)

    # UI exposes std devs; square them to get variances/covariances for NoiseConfig
    noise_cfg = NoiseConfig(
        imu_cov     = np.diag([float(imu_ax_std)**2, float(imu_ay_std)**2, float(gyro_std)**2]),
        pos_cov     = np.diag([float(pos_sx)**2, float(pos_sy)**2]),
        heading_var = float(hdg_std)**2,
    )

    sim = SIM_CLASSES[sim_type](
        mass=float(mass), inertia=float(inertia),
        noise_cfg=noise_cfg, dt=float(dt), n=int(n_trials),
        **sim_kw,
    )

    theta0_rad = float(theta0_deg) * np.pi / 180.0
    init_state = np.array([float(x0), float(y0), theta0_rad,
                            float(vx0), float(vy0), float(omega0)])

    traj = sim.simulate((0.0, float(t_final)), init_state)

    z_seq, u_seq = _build_ekf_inputs(traj)
    ekf = EKF(
        z0          = z_seq[:, 0, :],
        sigma0      = float(sigma0),
        dt          = float(dt),
        Sigma_imu   = noise_cfg.imu_cov,
        Sigma_pos   = noise_cfg.pos_cov,
        heading_var = noise_cfg.heading_var,
    )
    s_hist, P_hist = ekf.run(z_seq, u_seq)

    label = SIM_OPTIONS[sim_type]
    plots = [
        ("Trajectory",
         plot_trajectory(traj, title=f"{label} — Trajectory")),
        ("Trajectory Animation",
         animate_trajectory(traj, title=f"{label} — Animation", frame_duration_ms=30)),
        ("IMU Measurements",
         plot_imu_measurements(traj, title=f"{label} — IMU Measurements")),
        (f"MC Trajectories ±1σ ({n_trials} trials)",
         plot_trajectory_with_bounds(traj, title=f"{label} — MC Trajectories ±1σ")),
        ("EKF — Single Trial Animation",
         animate_estimate(traj, s_hist, P_hist, title=f"{label} — EKF Single Trial")),
        ("EKF — State Estimates",
         plot_ekf_states(traj, s_hist, title=f"{label} — EKF States")),
        (f"EKF — All MC Estimates ({n_trials} trials)",
         plot_mc_estimates(traj, s_hist, title=f"{label} — EKF Estimates vs GT")),
        (f"EKF — Uncertainty & Error ({n_trials} trials)",
         plot_mc_mse(P_hist, traj, s_hist, title=f"{label} — Uncertainty & Error")),
    ]

    children = []
    for heading, fig in plots:
        children.append(
            html.Div([
                html.H3(heading, className="section-title", style={"marginTop": "24px"}),
                dcc.Graph(figure=fig, config={"responsive": True}),
            ])
        )

    return children
