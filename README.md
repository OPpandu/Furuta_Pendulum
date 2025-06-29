# Furuta_Pendulum

A comprehensive project on the design, simulation, and control of a **rotary inverted pendulum** (Furuta Pendulum). This repository includes CAD files, hardware control scripts, control algorithm simulations, and reinforcement learning-based experiments.

## 📌 Project Overview

The Furuta Pendulum is a classic benchmark system used in control systems research and education. This project aims to:

- Build a physical model of the Furuta Pendulum using 3D-printed and off-the-shelf components.
- Implement and test control strategies such as LQR and PID in hardware.
- Simulate control algorithms to verify stability and performance.
- Used Energy Based swing-up control as a non linear way to swing the pendulum
- Implement a switch between energy based and LQR control at a certain threshold angle
- Exploring reinforcement learning (RL) approaches for pendulum stabilization and swing-up tasks.

---

## 📁 Repository Structure

```bash
├── CAD/                            # 3D design files for hardware
├── lqr-hardware-control/          # Code for LQR implementation on real hardware
├── rl-py-arduino(in progress)/    # Reinforcement learning (Python ↔ Arduino) setup
├── simulations-control/           # MATLAB/Python simulations for control strategies
├── made-for-science-...pdf        # Reference material on Furuta Pendulum (e.g., Quanser docs)
├── part_list.txt                  # Bill of materials (BOM)
├── README.md                      # This file
└── .DS_Store                      # macOS system file (can be ignored)