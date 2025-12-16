# Physics Simulation Module (C++)

This project is a **modular C++ physics simulation application** developed as a university module project.
It combines multiple classical physics problems into a single, menu-driven console program.

---

## 📌 Included Simulations

### 1️⃣ Projectile Motion
- Motion with and without air resistance
- Adjustable initial speed, launch angle, and height
- Numerical time-step simulation
- Optional collision detection with a person at a given position

### 2️⃣ Asteroid Collision
- Two-body collision simulation
- Final velocity calculations
- Kinetic energy loss computation
- Trajectory vector output

### 3️⃣ Ballistic Pendulum
- Momentum conservation
- Maximum height and angle calculation
- Energy analysis before and after collision
- Pendulum period calculation
- Automatic unit conversions (g → kg, cm → m)

### 4️⃣ Satellite Motion
- Orbital velocity calculation
- Orbital period calculation
- Based on gravitational physics
- Custom mass, radius, and altitude inputs

---

## 🧮 Physics Concepts Used
- Newton’s Laws of Motion  
- Kinematics  
- Momentum & Energy Conservation  
- Air Resistance (Drag Force)  
- Gravitational Force  
- Circular Orbital Motion  

---

## 🛠️ Technologies
- **Language:** C++
- **Libraries:** `<iostream>`, `<cmath>`, `<cctype>`

---

## ▶️ How to Run

Compile:
```bash
g++ "C++ Program.cpp" -o physics_simulation
