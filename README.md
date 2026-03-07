# Differential Drive Robot Simulator

A simple **2D differential drive robot simulator** built with **Python and Pygame** that models:

* motor dynamics
* proportional speed control
* robot kinematics
* telemetry visualization
* CSV data logging

The project was created to experiment with **control systems and robotics concepts** such as motor lag, heading drift, and feedback control.

---

# Features

### Motor Dynamics

The motors are modeled as a **first-order system**:

τ dv/dt = G·u − v

Where:

* `τ` = motor time constant
* `G` = motor gain
* `u` = controller command
* `v` = motor speed

This creates realistic motor behaviour where speeds **ramp gradually instead of changing instantly**.

---

### Differential Drive Kinematics

Robot motion is calculated using the standard differential drive equations:

v = (vL + vR) / 2
ω = (vR − vL) / wheel_base

Robot pose update:

x += v cos(θ) dt
y += v sin(θ) dt
θ += ω dt

---

### Speed Controller

Each wheel is controlled using a **proportional controller**:

command = Kc × error

Where:

error = target_speed − wheel_speed

This allows the wheels to attempt to reach a desired speed.

---

### Motor Mismatch Simulation

The simulator intentionally introduces **motor asymmetry**:

left_motor_gain ≠ right_motor_gain

This causes the robot to **naturally drift**, replicating behaviour seen in real robots.

This highlights the importance of **heading feedback control**.

---

### Telemetry Dashboard

The simulator includes a live telemetry panel showing:

* controller gains
* target speed
* wheel errors
* motor commands
* wheel speeds
* robot velocity
* angular velocity
* robot heading

---

### CSV Telemetry Logging

Simulation data is logged to CSV including:

* frame number
* time
* controller outputs
* wheel speeds
* robot velocity
* angular velocity
* robot position

This allows further analysis using tools like:

* Python
* Excel
* MATLAB

---

# Installation

Clone the repository:

```
git clone https://github.com/ABIMGOOD/CONTROL.git
cd 
```

Install dependencies:

```
pip install pygame
```

---

# Running the Simulator

```
python robot_sim.py
```

A window will open showing:

* robot motion
* heading direction
* motion trail
* live telemetry panel

---

# Future Improvements

Possible extensions:

* PID speed control
* heading stabilization controller
* wheel encoder simulation
* IMU / gyroscope simulation
* obstacle environments
* path following algorithms
* ROS integration

---

# Concepts Demonstrated

This project demonstrates several core robotics concepts:

* differential drive kinematics
* first-order motor dynamics
* feedback control
* proportional controllers
* heading drift caused by actuator mismatch
* simulation timestep integration

---

# Technologies

* Python
* Pygame
* CSV telemetry logging

---

# Author
Goodness Abimbola

Created as a learning project for exploring **robot motion and control systems**.

---
