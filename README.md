# Ragdoll Physics Simulator

A high-performance 2D physics engine developed as a 4th-year engineering project. This simulator focuses on articulated body dynamics (Ragdolls) using a robust constraint-based approach.

## Overview

This project features two distinct implementations:

- Python: Developed for rapid iteration, debugging, and visual validation of physical laws.

- C++: An optimized production-ready version focusing on memory efficiency and computation speed.

## Technical Architecture

The engine follows a modular pipeline designed for stability and precision:

### 1. Force Application

- **External Forces:** Global environmental effects like **Gravity** and dynamic **Wind** vectors.

- **Internal Forces:** Manual **Impulses** applied to specific bodies for user interaction or procedural animation.

### 2. Numerical Integration

The engine is built on the **Velocity Verlet** integration scheme. Unlike basic Euler, this model offers significantly better energy conservation and stability for complex constraints:

### 3. Constraints & Ragdoll Logic

To simulate articulated skeletons, we use **Distance Constraints** solved via iterative relaxation (similar to Jakobsen’s method). This ensures that bones maintain their length while allowing realistic movement.

### 4. Collision Handling

- **Detection:** Implementation of the **SAT (Separating Axis Theorem)** for precise convex polygon intersection.

- **Resolution:** Impulse-based resolution to handle penetration depth and kinetic energy transfer.

## Physics & Mathematics

- **Integration:** In-depth implementation of Velocity Verlet to minimize numerical drift.

- **Constraints:** Solving linear systems for rigid link stability.

- **SAT Method:** Detailed geometric projection to calculate the Minimum Translation Vector (MTV).

## Visual Examples

### a. Python

* Simple pendulum 

![](C:\Users\simon\Documents\ecole\ei4\s8\projet_info\Ragdoll-Physics-Engine\doc\examples_gif\python\python_pendulum.gif "simple pendulum")

* Drag

![](./doc/examples_gif/python/python_drag.gif)

* Impulsion

![](./doc/examples_gif/python/python_impulse.gif)

## Installation & Use

## Stretch Goals (Future Improvements)

## 
