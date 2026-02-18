from dash import dcc, html, register_page

register_page(__name__, path="/", name="Home")

cards = [
    {
        "title": "Control Simulation",
        "route": "/control-simulation",
        "is_live": True,
    },
]

layout = html.Main(
    [
        html.Section(
            [
                html.Span("Control Systems Suite", className="eyebrow"),
                html.H1("Controls Visualizations", className="hero-title"),
                html.P(
                    "A focused workspace for simulation, analysis, and controller prototyping.",
                    className="hero-subtitle",
                ),
            ],
            className="hero",
        ),
        html.Section(
            [
                html.Div(
                    [
                        html.H2(card["title"], className="card-title"),
                        (
                            dcc.Link(
                                html.Button("Control Simulation", className="primary-btn menu-btn"),
                                href=card["route"],
                                className="menu-link",
                            )
                            if card["is_live"]
                            else html.Button(
                                "Coming Soon", className="ghost-btn menu-btn", disabled=True
                            )
                        ),
                    ],
                    className="viz-card menu-row",
                )
                for card in cards
            ],
            className="card-grid",
        ),
    ],
    className="landing-page",
)
