"""
trajectory_generators.py: Module for generating reference trajectories for control systems.
For now we will only implement reference generation
TODO: Feed forward control and plant inversion will be added later
"""

import numpy as np
from abc import ABC, abstractmethod


class TrajectoryGenerator(ABC):
    """
    Abstract base class for trajectory generators.
    
    This class provides a stateless interface for generating reference trajectories.
    All trajectory generators should inherit from this class and implement the required methods.
    
    The class is designed to work with the controller input system, automatically generating
    input fields for each system output.
    """
    
    # Dictionary defining the input parameters needed for this trajectory type
    # Format: {"input_name": {"type": "number", "value": default_value, "description": "..."}}
    inputs = {}

    @classmethod
    def get_inputs(cls, input_info: list[str], output_info: list[str]) -> dict:
        """
        Generate input field definitions for the UI based on system outputs.
        
        Creates one input field for each combination of trajectory parameter and system output.
        For example, if the trajectory needs a "value" parameter and the system has 2 outputs,
        this will create "value_0" and "value_1" input fields.

        Args:
            input_info: List of input names for the system
            output_info: List of output names for the system

        Returns:
            Dictionary of input field definitions compatible with the controller make_input_field method.
        """
        ui_inputs = {}
        
        # Repeat the inputs for every dimension of the target output
        for i, output_name in enumerate(output_info):
            # Loop over each parameter
            for input_name, props in cls.inputs.items():
                field_name = f"{input_name}_{i}"
                ui_inputs[field_name] = {
                    "type": props["type"],
                    "value": props["value"],
                    "description": f"{props['description']} (Output: {output_name})"
                }
        
        return ui_inputs

    @classmethod
    @abstractmethod
    def generate(cls, A: np.ndarray, B: np.ndarray, C: np.ndarray, 
                t: np.ndarray, trajectory_inputs: dict) -> np.ndarray:
        """
        Generate the complete reference trajectory for all time points.

        This is a stateless method that generates the entire trajectory array
        based on the system matrices and input parameters.

        Args:
            A: System state matrix (n x n)
            B: System input matrix (n x m) 
            C: System output matrix (p x n)
            t: Time array of shape (N,)
            trajectory_inputs: Dictionary containing trajectory parameters from UI

        Returns:
            Reference trajectory array of shape (N, p) where:
            - N is the number of time points
            - p is the number of system outputs
        """
        raise NotImplementedError("This method should be overridden by subclasses.")


class ConstantTrajectoryGenerator(TrajectoryGenerator):
    """
    Trajectory generator that produces constant reference trajectories.
    
    Each system output can have a different constant reference value.
    """

    inputs = {
        "value": {
            "type": "number",
            "value": 0.0,
            "description": "Constant reference value"
        }
    }

    @classmethod
    def generate(cls, A: np.ndarray, B: np.ndarray, C: np.ndarray, 
                t: np.ndarray, trajectory_inputs: dict) -> np.ndarray:

        N = len(t)
        p = C.shape[0]  # number of outputs
        
        # Initialize trajectory array
        trajectory = np.zeros((N, p))
        
        # Fill each output with its constant value
        for i in range(p):
            value_key = f"value_{i}"
            if value_key in trajectory_inputs:
                trajectory[:, i] = trajectory_inputs[value_key]
        
        return trajectory


class StepTrajectoryGenerator(TrajectoryGenerator):
    """
    Trajectory generator that produces step reference trajectories.
    
    Each output starts at an initial value and steps to a final value at a specified time.
    """

    inputs = {
        "initial_value": {
            "type": "number",
            "value": 0.0,
            "description": "Initial reference value"
        },
        "final_value": {
            "type": "number", 
            "value": 1.0,
            "description": "Final reference value after step"
        },
        "step_time": {
            "type": "number",
            "value": 1.0,
            "description": "Time at which step occurs"
        }
    }

    @classmethod
    def generate(cls, A: np.ndarray, B: np.ndarray, C: np.ndarray,
                t: np.ndarray, trajectory_inputs: dict) -> np.ndarray:
        N = len(t)
        p = C.shape[0]  # number of outputs
        
        # Initialize trajectory array
        trajectory = np.zeros((N, p))
        
        # Generate step trajectory for each output
        for i in range(p):
            initial_key = f"initial_value_{i}"
            final_key = f"final_value_{i}"
            step_time_key = f"step_time_{i}"
            
            initial_value = trajectory_inputs.get(initial_key, 0.0)
            final_value = trajectory_inputs.get(final_key, 1.0)
            step_time = trajectory_inputs.get(step_time_key, 1.0)
            
            # Create step function
            trajectory[:, i] = np.where(t >= step_time, final_value, initial_value)
        
        return trajectory


class SinusoidalTrajectoryGenerator(TrajectoryGenerator):
    """
    Trajectory generator that produces sinusoidal reference trajectories.
    
    Each output follows a sinusoidal pattern with configurable amplitude, frequency, and phase.
    """

    inputs = {
        "amplitude": {
            "type": "number",
            "value": 1.0,
            "description": "Amplitude of sinusoidal trajectory"
        },
        "frequency": {
            "type": "number",
            "value": 1.0,
            "description": "Frequency in Hz"
        },
        "phase": {
            "type": "number",
            "value": 0.0,
            "description": "Phase offset in radians"
        },
        "offset": {
            "type": "number",
            "value": 0.0,
            "description": "DC offset"
        }
    }

    @classmethod
    def generate(cls, A: np.ndarray, B: np.ndarray, C: np.ndarray,
                t: np.ndarray, trajectory_inputs: dict) -> np.ndarray:
        N = len(t)
        p = C.shape[0]  # number of outputs
        
        # Initialize trajectory array
        trajectory = np.zeros((N, p))
        
        # Generate sinusoidal trajectory for each output
        for i in range(p):
            amplitude_key = f"amplitude_{i}"
            frequency_key = f"frequency_{i}"
            phase_key = f"phase_{i}"
            offset_key = f"offset_{i}"
            
            amplitude = trajectory_inputs.get(amplitude_key, 1.0)
            frequency = trajectory_inputs.get(frequency_key, 1.0)
            phase = trajectory_inputs.get(phase_key, 0.0)
            offset = trajectory_inputs.get(offset_key, 0.0)
            
            # Generate sinusoidal trajectory
            trajectory[:, i] = amplitude * np.sin(2 * np.pi * frequency * t + phase) + offset
        
        return trajectory