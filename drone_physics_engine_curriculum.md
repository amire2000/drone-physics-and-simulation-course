# Building a Drone Physics Engine from Scratch

*A Comprehensive Tutorial Curriculum using PyBullet & PyTorch*

## Course Overview

This curriculum is designed to guide software engineers, robotics enthusiasts, and students through the complete process of building a mathematically rigorous, real-time drone physics simulation from the ground up using **PyBullet**. By avoiding heavy external dependencies like Betaflight SITL, learners develop an intuitive understanding of the absolute core physics, aerodynamic forces, and electrical layers powering modern quadcopters. The ultimate objective is to map high-level decision algorithms (such as continuous multi-variable “System 1” MLP decision models) to low-level controls, culminating in a fully autonomous flight loop capable of performing a stable takeoff and precision hover at a targeted altitude.

## Module 1: The Absolute Basics – Mass and External Forces

Establishes the foundation of rigid body dynamics inside the virtual sandbox. Students initialize the environment and understand how continuous force components interact with static masses under gravity.

- **Topic 1.1: Initializing the PyBullet Sandbox** – Setting up the physics client, gravity vector `(0, 0, -9.81)`, and defining the default 240 Hz physics tick rate environment.
- **Topic 1.2: The Free-Falling Body** – Spawning basic structures, analyzing relative linear tracking velocity, and verifying real-world physical acceleration constants.
- **Topic 1.3: Counter-Acting Gravity** – Applying raw upward vectors using `applyExternalForce` to calculate the exact structural equilibrium required for uniform hover physics.

## Module 2: The Physical Asset Blueprint (URDF Engine)

Transitions from uniform geometric boxes to structured, highly accurate drone physical frames. Introduces mechanical engineering parameters directly to the simulation sandbox.

- **Topic 2.1: URDF Architecture Definition** – Structural layout configuration using explicit XML notation, scaling components to a typical 650 g 5-inch midquad mass limit.
- **Topic 2.2: Moments of Inertia Matrix Math** – Deriving analytical tensor parameters (`Ixx`, `Iyy`, `Izz`) by modeling a core solid central fuselage combined with four distinct motor point masses.
- **Topic 2.3: Spawning & Integrity Evaluation** – Instantiating custom `.urdf` assets and assessing real-time structural responses to asymmetric off-axis torque forces.

## Module 3: True Propeller Dynamics & Aerodynamics

Replaces crude force assumptions with fluid dynamics equations. Models the unique thrust and drag profiles of physical propellers moving through standard air columns.

- **Topic 3.1: Propeller Geometry Fundamentals** – Mapping physical specifications (Diameter, Pitch, Blade Count) directly to abstract fluid performance coefficients.
- **Topic 3.2: Mapping PWM to Thrust Vectors (Kt)** – Implementing the quadratic thrust scaling formula based on real-world test data benchmarks from the UIUC Propeller database.
- **Topic 3.3: Rotational Aerodynamic Torque (Yaw Dynamics)** – Incorporating the counter-rotational drag torque profiles of alternating Clockwise (CW) and Counter-Clockwise (CCW) motor configurations.
- **Topic 3.4: Simulating Environmental Air Drag** – Injecting a continuous fluid resistance vector acting opposite to the current global velocity to simulate atmospheric drag forces.

## Module 4: Low-Level Flight Control – The Motor Mixer & PID Engine

Builds the software layer responsible for vehicle attitude stabilization. Emulates traditional flight controller firmware pipelines within the loop layout.

- **Topic 4.1: The Motor Mixer Matrix** – Blending abstract joystick directional commands into explicit proportional outputs tailored for an industrial X-configuration frame layout.
- **Topic 4.2: Rate PID Controller Emulation** – Building a closed-loop Proportional-Integral-Derivative algorithm inside Python to actively counter external physical displacement.
- **Topic 4.3: Synthetic Joystick Signal Injection** – Normalizing inputs to standard 1000–2000 PWM boundaries to bridge high-level targets with real low-level parameters.

## Module 5: The Electrical Layer – Battery Profiles & Voltage Sag

Introduces physical real-world performance decay to the simulation environment by modeling true electrochemical constraints under intensive load.

- **Topic 5.1: LiPo Cell State-of-Charge Dynamics** – Simulating true discharge boundaries ranging from 4.2 V down to a critical nominal floor of 3.7 V per cell.
- **Topic 5.2: Voltage Sag & Internal Resistance Math** – Leveraging Ohm’s Law to dynamically throttle available voltage benchmarks relative to instant operational current consumption spikes.
- **Topic 5.3: Performance Scaling Decay** – Programmatically mapping maximum available thrust limits to sagging cell potentials, exposing flight handling degradation over time.

## Module 6: Capstone Project – Autonomous Takeoff and Precision Hover

The ultimate integration challenge. Students deploy a pre-trained or structured Multi-Layer Perceptron (MLP) “Jev-style” decision matrix to smoothly guide a dead-stop drone up to an exact 5-meter hover plane.

- **Topic 6.1: Closed-Loop Altitude Tracking Logic** – Mapping spatial displacement data directly to continuous neural inputs.
- **Topic 6.2: High-Level to Low-Level Signal Mapping** – Processing live MLP inferences down to safe 1000–2000 PWM mixer parameters.
- **Topic 6.3: Multi-Layer System Verification** – Balancing battery depletion, aerodynamic drag offsets, and inertia constraints to maintain a rock-steady automated hover state.
