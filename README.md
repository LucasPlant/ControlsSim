---
title: Control Systems Visualization
emoji: 🦿
colorFrom: blue
colorTo: red
sdk: docker
sdk_version: 3.0.0
app_file: control_vis_app.py
pinned: true
---


# ControlSim

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A web-based dynamic systems and control simulation platform that enables users to visualize and interact with various control systems in real-time. Built with Python using the Dash framework and Plotly for interactive visualizations.

## Features

- Interactive system simulation in your web browser
- Real-time visualization of system dynamics
- Multiple controller implementations (PID, State Feedback, etc.)
- Various dynamic system models
- Built on top of modern Python frameworks (Dash, Plotly)

## Getting Started

### Prerequisites

- Python 3.x
- Git

### Installation

1. Clone the repository:
   ```bash
   git clone git@github.com:LucasPlant/ControlsSim.git
   cd ControlsSim
   ```

2. Install dependencies:
   ```bash
   # TODO: Add requirements.txt or Docker setup
   ```

3. Run the application:
   ```bash
   python control_vis_app.py
   ```

4. Open your web browser and navigate to the localhost address shown in the terminal.

## Contributing

Contributions are welcome! If you'd like to contribute please reach out to me

## Roadmap

### High Priority
- [ ] Define and complete MVP features
- [ ] Deploy web application
- [ ] Create requirements.txt or Docker container

### Nice to have
- [ ] Unit testing

### Analysis Features
- [ ] Root Locus plots
- [ ] Other classical control plots (Nyquist)
- [ ] Poles and zeros visualization for PID
- [ ] Transfer Functions and Bode Plots
- [ ] Controllable subspaces analysis
- [ ] Kalman decompositions

### System Models
- [ ] Cart pole system
- [ ] Two-mass spring system
- [ ] 2D/3D mass with force
  - [ ] Orientation and attitude
  - [ ] COSMOS
  - [ ] Thruster allocation
- [ ] Inverted Pendulum / Cart pole
- [ ] Cruise control simulation
- [ ] Multiple pendulum systems
- [ ] Orbital body simulation

### User Interface
- [ ] Controller state initialization
- [ ] State persistence
- [ ] UI organization and beautification

### Control Systems
- [ ] MIMO Controllers
  - [ ] Array dimension compatibility
  - [ ] MIMO FSFB using pole placement
  - [ ] Mode shaping
- [ ] Optimal Controllers
  - [ ] LQR (Finite/Infinite time horizon)
  - [ ] Kalman Filter
- [ ] Feed Forward / Trajectory Planning
  - [ ] Plant inversion
  - [ ] Polynomial trajectory planning
  - [ ] Optimal trajectories
  - [ ] Command shaping

### Advanced Features
- [ ] Stochastic Elements
  - [ ] Sensor noise simulation
  - [ ] Actuator noise
  - [ ] Disturbance rejection
- [ ] Propagator improvements
  - [ ] Controller update period

## Development Guidelines
- Maintain comprehensive inline documentation
- Use type hints for better code maintainability

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
