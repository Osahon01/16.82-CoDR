# Aircraft Design & Mass Closure Optimization

Aircraft sizing and optimization framework for preliminary hybrid-electric aircraft design. The repository combines physics-based subsystem models in Python with a high-performance C++ optimization backend.

---

## Overview

This project implements a Conceptual Design and Optimization Routine (CoDR) for a 9-passenger hybrid-electric regional aircraft. It couples aerodynamic, structural, propulsion, and mission models to enforce mass closure and evaluate design feasibility across a constrained design space.

---

## Repository Structure


### Python (`82_py/`)

- Aircraft and subsystem models
- Mass closure optimization
- Mission performance analysis
- Result storage and visualization

### C++ (`82_cpp_opt/`)

- High-performance optimizer
- Fitness and constraint evaluation
- Large-scale design sweeps

