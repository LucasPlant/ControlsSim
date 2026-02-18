import os

from dash import Dash, html, page_container

LOCAL = False

app = Dash(
    __name__,
    use_pages=True,
    suppress_callback_exceptions=True,
    external_stylesheets=[
        "https://fonts.googleapis.com/css2?family=Manrope:wght@400;500;600;700;800&family=Space+Grotesk:wght@500;600;700&display=swap"
    ],
    assets_folder="assets",
)

app.index_string = """<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <title>{%title%}</title>
        {%favicon%}
        {%css%}
        <link rel="stylesheet" href="/assets/style.css">
        <meta name="viewport" content="width=device-width, initial-scale=1">
    </head>
    <body>
        {%app_entry%}
        <footer>
            {%config%}
            {%scripts%}
            {%renderer%}
        </footer>
    </body>
</html>"""

app.layout = html.Div(page_container, className="app-shell")

server = app.server

if __name__ == "__main__":
    port = int(os.getenv("PORT", 7860))
    if LOCAL:
        app.run(debug=True, host="127.0.0.1", port=port)
    else:
        app.run(debug=False, host="0.0.0.0", port=port)
