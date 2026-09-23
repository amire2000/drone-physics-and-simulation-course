# Drone Physics Fundamentals — Structured Lesson Plan

**Target:** Students with basic Python and introductory algebra/calculus  
**Format:** Theory + Python experiments + visualization  
**Estimated total:** ~30–36 hours  
**Final outcome:** A small configurable quadcopter physics simulator.

---

## Module 1 — Foundations: Forces, Motion & Reference Frames

**Estimated time: 4–5 hours**

### Objectives

Students should be able to explain the physical quantities needed to describe drone motion and distinguish between the **world/inertial frame** and **drone/body frame**.

They should understand the relationship

\[
\mathbf F = m\mathbf a
\]

and identify the main forces acting on a quadcopter: gravity, rotor thrust and aerodynamic forces.

### Key topics

Vectors and coordinate systems; position, velocity and acceleration; mass and gravity; Newton's laws; force versus torque; center of mass; world frame versus body frame; basic transformations between frames.

Introduce the drone state:

\[
\mathbf p=[x,y,z]
\]

\[
\mathbf v=[v_x,v_y,v_z]
\]

and initially keep attitude simple.

### Suggested activities

Start with a **1-D falling drone simulation**:

\[
F_z=-mg
\]

then integrate acceleration to obtain velocity and position.

Next add an upward force representing total propeller thrust:

\[
F_z=T-mg
\]

Students experiment with:

\[
T<mg,\qquad T=mg,\qquad T>mg
\]

and discover physically what corresponds to descending, hovering and climbing.

**Mini-lab:** Write a Python program that simulates a 1 kg drone for 10 seconds and plots altitude and vertical velocity.

---

## Module 2 — Rigid-Body Motion & Rotation

**Estimated time: 5–6 hours**

### Objectives

Students move from treating the drone as a point mass to treating it as a **6-DOF rigid body**.

They should understand roll, pitch and yaw and why rotational motion requires torque and moment of inertia.

### Key topics

Six degrees of freedom; roll/pitch/yaw; angular velocity

\[
\boldsymbol\omega=[p,q,r]
\]

torque

\[
\boldsymbol\tau=[\tau_x,\tau_y,\tau_z]
\]

moment of inertia and the inertia tensor.

Introduce Euler's rigid-body equation:

\[
I\dot{\boldsymbol\omega}
+
\boldsymbol\omega\times(I\boldsymbol\omega)
=
\boldsymbol\tau
\]

Then introduce rotation matrices and, conceptually, quaternions.

### Suggested activities

Use a simplified drone consisting of a central body plus four motor masses.

Students estimate \(I_x,I_y,I_z\), apply a known torque and calculate the expected angular acceleration.

Then simulate:

```text
torque
   ↓
angular acceleration
   ↓
angular velocity
   ↓
attitude
```

**Mini-lab:** Apply a constant roll torque for 0.5 seconds and plot roll angle and roll rate.

---

## Module 3 — Motors, Propellers, Thrust & Torque

**Estimated time: 5–6 hours**

This is where the model starts behaving like a quadcopter rather than a generic rigid body.

### Objectives

Students should understand how four motors generate both **linear force and rotational torque**, and how changing individual motor speeds creates throttle, roll, pitch and yaw.

### Key topics

Motor RPM/angular velocity; propeller thrust; reaction torque; CW versus CCW propellers; arm length; motor placement; motor mixing.

Introduce the simplified rotor model:

\[
T_i=k_T\omega_i^2
\]

and reaction torque:

\[
Q_i=k_Q\omega_i^2
\]

Then show how four motors combine:

\[
T=\sum_{i=1}^{4}T_i
\]

and how an off-center thrust force generates torque:

\[
\boldsymbol\tau=\mathbf r\times\mathbf F
\]

### Suggested activities

Create a virtual quad:

```text
        M1
         ↑
         |
   M4 ← CoM → M2
         |
         ↓
        M3
```

Give students four sliders representing motor RPM.

Have them predict the result before running the simulation:

```text
all motors ↑        → climb
left motors ↑       → roll
rear motors ↑       → pitch
CW pair ↑           → yaw
```

The exact signs depend on the selected motor numbering and coordinate convention.

**Mini-lab:** Build a Python motor mixer that converts four motor speeds into total thrust and \((\tau_x,\tau_y,\tau_z)\).

---

## Module 4 — Aerodynamics, Drag & Environmental Effects

**Estimated time: 4–5 hours**

### Objectives

Students should understand why the simple Newtonian model isn't enough for realistic flight and learn to introduce environmental forces progressively rather than attempting full CFD.

### Key topics

Air density; aerodynamic drag; wind-relative velocity; projected area; drag coefficient; propeller aerodynamic effects; disturbances; optionally ground effect.

Introduce:

\[
F_D=
\frac12\rho C_D A v^2
\]

with the force acting opposite relative airflow.

An important concept is relative air velocity:

\[
\mathbf v_{\text{air}}
=
\mathbf v_{\text{drone}}
-
\mathbf v_{\text{wind}}
\]

so wind is not simply an arbitrary force added to the drone.

### Suggested activities

Take the previous quadcopter simulator and add drag.

Run the same experiment twice:

```text
Simulation A
drag = 0

Simulation B
drag enabled
```

Compare acceleration and terminal behavior.

Next introduce a 5 m/s crosswind and observe lateral displacement.

**Mini-lab:** Drop the same virtual drone with and without drag and compare the velocity curves.

---

## Module 5 — Numerical Integration & Building the Physics Engine

**Estimated time: 5–6 hours**

### Objectives

Students should understand how continuous differential equations become a discrete simulation running at, for example, **100–1000 Hz**.

The goal is to turn the equations from Modules 1–4 into an actual physics engine.

### Key topics

Simulation timestep \(\Delta t\); state variables; Euler integration; semi-implicit Euler; Runge-Kutta methods; numerical stability; simulation rate; force accumulation; torque accumulation.

Start with:

\[
\mathbf v_{k+1}
=
\mathbf v_k+\mathbf a_k\Delta t
\]

\[
\mathbf p_{k+1}
=
\mathbf p_k+\mathbf v_k\Delta t
\]

and then discuss why numerical methods matter.

### Suggested activities

Build the first complete update loop:

```text
motor commands
      ↓
motor model
      ↓
thrust + torque
      ↓
gravity + drag + wind
      ↓
Σ forces / Σ torques
      ↓
linear + angular acceleration
      ↓
numerical integrator
      ↓
new drone state
```

Run the same scenario with:

\[
\Delta t=0.1,\;0.01,\;0.001
\]

and compare the results.

This makes numerical instability visible rather than merely explaining it mathematically.

**Mini-lab:** Implement Euler integration first, then compare it with RK4 for the same flight maneuver.

---

## Module 6 — Complete Drone Physics Engine & Validation

**Estimated time: 7–8 hours**

This should be the capstone rather than another isolated theory module.

### Objectives

Students combine everything into a configurable drone model and learn that a physics engine must be **validated**, not merely made to "look realistic."

### Key topics

Full drone state; mass; inertia tensor; motor positions; motor KV/RPM limits; thrust coefficients; propeller direction; gravity; aerodynamic drag; wind; simulation timestep; sensor-ready state outputs.

The engine should expose something approximately like:

\[
\mathbf x =
[
\mathbf p,
\mathbf v,
\mathbf q,
\boldsymbol\omega
]
\]

where \(\mathbf q\) represents attitude as a quaternion.

### Suggested capstone

Students implement:

```text
DroneModel
 ├── Mass
 ├── Inertia
 ├── Motor[4]
 ├── Propeller[4]
 ├── Aerodynamics
 └── State

        ↓

PhysicsEngine
 ├── calculate_gravity()
 ├── calculate_motor_thrust()
 ├── calculate_motor_torque()
 ├── calculate_drag()
 ├── calculate_wind()
 ├── calculate_acceleration()
 └── integrate()
```

Then validate it with controlled experiments.

### Test 1 — Gravity

Motors off → drone falls at approximately \(g\).

### Test 2 — Hover

Find motor speed such that:

\[
\sum T_i=mg
\]

The drone should approximately maintain altitude.

### Test 3 — Vertical acceleration

Increase all motors equally.

### Test 4 — Roll

Change left/right motor thrust and verify the expected roll acceleration.

### Test 5 — Pitch

Change front/rear thrust.

### Test 6 — Yaw

Change CW/CCW motor pairs.

### Test 7 — Wind

Apply a crosswind and observe drift.

---

## Recommended Course Progression

I would structure the whole course around one simulator that becomes progressively more capable:

\[
\boxed{\text{1-D particle}}
\rightarrow
\boxed{\text{3-D particle}}
\rightarrow
\boxed{\text{rigid body}}
\rightarrow
\boxed{\text{4 motors}}
\rightarrow
\boxed{\text{aerodynamics}}
\rightarrow
\boxed{\text{complete quadcopter}}
\]

That progression is especially useful for the eventual goal of connecting the physics engine to **Betaflight SITL**. The physics engine can ultimately sit underneath the flight controller:

```text
                Betaflight SITL
                      │
                motor commands
                      ↓
              ┌───────────────┐
              │ Physics Engine│
              │               │
              │ motors        │
              │ propellers    │
              │ rigid body    │
              │ aerodynamics  │
              │ environment   │
              └───────┬───────┘
                      │
                simulated state
                      ↓
              ┌───────────────┐
              │ Sensor Models │
              │ IMU           │
              │ Barometer     │
              │ Camera pose   │
              └───────┬───────┘
                      │
                      └────→ Betaflight
```

This gives the six-module course a concrete destination: **by Module 6 the student isn't just learning equations—they have built the core physics model needed for a small FPV-drone simulator.**

The next natural course layer would then be sensor simulation, Betaflight SITL integration, and eventually coupling the physics state to a realistic visual renderer.
