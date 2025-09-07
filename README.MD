TODO:

Determine an MVP and once it is presentable then deploy to web

Plots:
    More analysis plots
        Root Locus
        Controlled poles and zeros for PID

        Transfer Functions / Bode Plots *

        Controllable subspaces
        Kalman decompositions

Systems:
    2 mass spring *

    2D / 3D mass with force
        add in orientation / attitude
        COSMOS
        Thruster allocation

    Inverted Pendulum / Cart pole *
    Cruise control
    Double / Triple inverted pendulum Cart pole
    
    Orbital bodies
        may require removing support for controllers for now (noop controller)

UI:
    Controller state initialization
    Persistence
    Some organization / beatification

Controllers:
    MIMO controllers *
        Checking all array dimensions to be compatible
        MIMO FSFB using pole placement
        Mode shaping?
    Optimal Controller *
        LQR
            Finite time horizon
            Infinite time horizon *
        Kalman Filter
    Feed Forward / Trajectory Planner
        Plant inversion
        Polynomial trajectory planning
        Optimal trajectories
        Command shaping

Stochasticity
    Noisy sensors
    Noisy effectors
    Disturbance rejection

Propagator improvements
    Runge Kutta 4th order *
    Controller update period *

unit testing if this ever becomes multi person 
    leaving this out for now as its just me and i dont have the time

Try to keep up with inline docs and typehints to futureproff