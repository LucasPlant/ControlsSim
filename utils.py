"""
utils.py: Utility functions for creating Dash input fields.

This module provides common functions for creating consistent input fields
across the application, reducing code duplication.
"""

from dash import html, dcc
from typing import Optional


def make_input_field(name: str, props: dict, source: str = "controller", 
                    existing_values: Optional[dict] = None) -> html.Div:
    """
    Create a standardized input field based on its type and properties.
    
    This function creates consistent input fields for dropdowns and other input types
    that can be used across systems, controllers, and trajectory generators.
    
    Args:
        name: The name/id for the input field
        props: Dictionary containing field properties including:
            - "type": The input type ("dropdown", "number", "text", etc.)
            - "value": Default value
            - "description": Label text for the field
            - "options": List of options (for dropdown type only)
        source: The source category for the input ("system", "controller", "trajectory_generator")
        existing_values: Dictionary of existing values to use instead of defaults
    
    Returns:
        html.Div containing the labeled input field
    """
    if existing_values is None:
        existing_values = {}
    
    # Get the value to use (existing value or default)
    value = existing_values.get(name, props["value"])
    
    if props["type"] == "dropdown":
        return html.Div(
            [
                html.Label(props["description"]),
                dcc.Dropdown(
                    id={"type": "input", "source": source, "name": name},
                    options=[
                        {"label": k, "value": k} for k in props["options"]
                    ],
                    value=value,
                    clearable=False,
                ),
            ]
        )
    else:
        return html.Div(
            [
                html.Label(props["description"]),
                dcc.Input(
                    id={"type": "input", "source": source, "name": name},
                    type=props["type"],
                    value=value,
                    debounce=True,
                ),
            ]
        )


def make_state_input_field(name: str, props: dict, source: str = "system", 
                          existing_values: Optional[dict] = None) -> list:
    """
    Create input fields specifically for state initialization.
    
    This function creates labeled input fields for system state initialization,
    following the pattern used in BaseSystem.
    
    Args:
        name: The name/id for the input field
        props: Dictionary containing:
            - "name": Display name for the state
            - "description": Description of the state
            - "value": Default value
        source: The source category for the input
        existing_values: Dictionary of existing values to use instead of defaults
    
    Returns:
        List containing [html.Label, dcc.Input] for the state field
    """
    if existing_values is None:
        existing_values = {}
    
    value = existing_values.get(name, props["value"])
    
    return [
        html.Label(props["name"] + ": " + props["description"]),
        dcc.Input(
            id={"type": "input", "source": source, "name": name},
            type="number",
            value=value,
            debounce=True,
        ),
    ]