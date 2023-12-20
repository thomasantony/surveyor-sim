# Surveyor Lander Simulator

This a simulation of the [Surveyor Lunar Lander](https://en.wikipedia.org/wiki/Surveyor_program) written in Rust, using the Bevy game engine. This is still a work-in-progress and actively in development. At the time of writing, the main entry-point into the program is in the `surveyor-graphics` crate.

The [landing algorithm](https://github.com/thomasantony/surveyor) has not been implemented yet and sensor/actuator systems are currently being built up along with the guidance and control software.

The goal of this project is to build a generic simulator in Rust that can be run natively or on the web, and used for simulating real-life spacecraft guidance algorithms.

## Building and Running

### Native
```
cargo run
```

### Web

Install [`trunk`](https://trunkrs.dev/) and run

```
trunk serve surveyor-graphics/index.html
```

## Project Status

This is currently heavily under development and not in an MVP state yet. Expect breaking changes, force-pushes to `master` and overall mayhem. I may also go into hibernation for random periods of time if I get too busy at my day-job or other pursuits.

### Current Features

- **Entity-Component-System (ECS)** architecture
    - Clear separation between the simulation (aka "truth-side") and the flight software
    - uses bevy "Events" for communication between dfferent systems and between simulation and flight software
- **3D Graphics visualization** using the [bevy](https://bevyengine.org/) game engine
    - Implemented in the `surveyor-graphics` crate
    - Uses 3D meshes from [Orbiter](http://orbit.medphys.ucl.ac.uk/index.html) for the spacecraft
- **6DOF Spacecraft Simulation** with Sensors and Actuators
    - Implemented in the `surveyor-physics` crate
    - Configurable using XML (investigating other options as well)
    - RK4 fixed-step integrator (see `surveyor-physics` crate)
    - 6DOF spacecraft dynamics w/ point-mass gravity model
        - Rudimentary collision detection (configurable) with planetary bodies
        - High-precision timing using the [hifitime](https://docs.rs/hifitime) crate
        - Earth and Moon positions loaded from JPL ephemerides (de440s.bsp) using [ANISE](https://github.com/nyx-space/anise)
    - Actuator models
        - Reaction Control System (RCS)
        - Vernier Engines with Thrust Vector Control
    - Sensor models (currently do not incorporate noise)
        - Bare-bones Gyroscope
        - Star Tracker that directly measures inertial attitude
        - Star Sensor (the original Surveyor had a [Canopus](https://en.wikipedia.org/wiki/Canopus) star sensor)
    - Interface between Simulation and Guidance software (`surveyor-physics/src/interfaces/`)
    - Precise control of simulation update vs flight software update (w/ the former running at least 2x faster)

- **Flight Software** (primarily just the Guidance, Navigation and Control aka GNC part)
    - Implemented in the `surveyor-gnc` crate
    - Three "Guidance Modes"
        - Idle
        - Detumble
        - Pointing (with a quaternion and/or body rate target)
    - Sensor components that convert from sensor-frame to body-frame
        - IMU
        - Star Tracker
        - Star Sensor
    - Actuator components that sends commands to the truth-side
        - RCS
        - Vernier engines w/ TVC
    - Sensor Aggregator
        - Keeps track of all sensor data, their health and provides data persistence in case of sensor outage
    - Attitude Estimator
        - Very simple component that directly passes through attitude data from the star tracker for now
    - Attitude Controller
        - Simple PID controller that can track a body-rate or inertial quaternion target
    - Control Allocator
        - Passes torque commands from the Attitude Controller to RCS
        - Will include commands to Thrust Vector system in the future
    - RCS Controller
        - Allocates body-frame torque request to RCS thruster duty cycles based on thruster position and orientation


### TODO:

- Load planetary data (such as radius and gravity parameters) from SPICE kernel
- Sun sensor simulation + GNC component
    - Requires feeding in position of Sun from Ephemerides
- Attitude estimator that uses Sun Sensor and Canopus Star Sensor to determine attitude
- TVC/Vernier Engine Attitude Controller
- Add particle effects/other visualization for RCS thrusters and vernier engines.
- Add solid retro rocket system (truth-side and FSW) and implement stage-separation logic
- Implement surveyor guidance algorithm from [here](https://github.com/thomasantony/surveyor) and demonstrate
