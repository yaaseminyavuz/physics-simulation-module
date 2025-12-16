# Physics Simulation Module (C++)

This project is a **modular physics simulation application** developed in **C++** as part of a university-level module project.  
The application integrates multiple classical physics problems into a single, menu-driven console program, aiming to demonstrate the practical implementation of theoretical physics concepts through numerical simulation.

---

## Abstract

The purpose of this project is to model and simulate fundamental physical systems using computational methods.  
The program allows users to interactively explore different physical phenomena, including projectile motion, collisions, pendulum dynamics, and satellite motion.  
Each simulation is implemented as an independent module, emphasizing modular programming, numerical accuracy, and user-driven parameter selection.

---

## Implemented Simulations

### 1. Projectile Motion Simulation
This module simulates the motion of a projectile launched from a given height with a specified initial velocity and angle.  
Both ideal motion (without air resistance) and non-ideal motion (with air resistance) are considered.

Key features:
- Adjustable initial conditions (velocity, angle, height)
- Numerical time-stepping approach
- Air resistance modeled using a quadratic drag force
- Optional collision detection with a person located at a specified position

---

### 2. Asteroid Collision Simulation
This module models a two-body collision between asteroids and calculates their post-collision velocities based on momentum conservation principles.

Key features:
- Final velocity computation for both bodies
- Calculation of the fraction of kinetic energy dissipated during the collision
- Trajectory vector representation of post-collision motion

---

### 3. Ballistic Pendulum Simulation
The ballistic pendulum module analyzes an inelastic collision between a projectile and a pendulum system.

Key features:
- Momentum conservation at the moment of impact
- Maximum height and angular displacement calculation
- Energy comparison before and after the collision
- Period calculation of the pendulum
- Automatic unit conversion for user inputs

---

### 4. Satellite Motion Simulation
This module calculates the orbital velocity and period of a satellite revolving around a central body under gravitational attraction.

Key features:
- Circular orbit approximation
- Orbital velocity computation
- Orbital period determination
- Customizable mass, radius, and altitude parameters
- Earth-based example for reference

---

## Physics Principles Applied

- Newton’s Laws of Motion  
- Kinematic Equations  
- Conservation of Momentum  
- Conservation of Energy  
- Gravitational Force  
- Circular Orbital Motion  
- Drag Force Modeling  

---

## Numerical Methods

The simulations employ numerical integration techniques using small time-step increments to approximate continuous motion.  
This approach enables the modeling of complex systems such as air resistance, where analytical solutions are not easily obtainable.

---

## Technologies Used

- **Programming Language:** C++
- **Standard Libraries:**  
  - `<iostream>`  
  - `<cmath>`  
  - `<cctype>`  

---

## Compilation and Execution

To compile the program:

```bash
g++ "C++ Program.cpp" -o physics_simulation

---

## References and Resources

The theoretical background and formulas used in this project are based on standard undergraduate-level physics and mechanics resources, including:

1. Halliday, D., Resnick, R., & Walker, J.  
   *Fundamentals of Physics*, Wiley.

2. Serway, R. A., & Jewett, J. W.  
   *Physics for Scientists and Engineers*, Cengage Learning.

3. Taylor, J. R.  
   *Classical Mechanics*, University Science Books.

4. NASA Glenn Research Center  
   Drag Force and Air Resistance Models  
   https://www.grc.nasa.gov

5. OpenStax  
   *College Physics*  
   https://openstax.org

These resources were used as references for classical mechanics principles, projectile motion, collision analysis, pendulum dynamics, and satellite orbital motion.
