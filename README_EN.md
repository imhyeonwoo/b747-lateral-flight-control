<div align="center">

# b747-lateral-flight-control

**B-747 lateral linear-model-based flight controller design & simulation (MATLAB/Simulink)**

</div>

---

## One-line summary
With the goal of **stabilizing lateral dynamics** for rudder/aileron inputs,  
this project designs and compares **Classical control (lead/lag + wash-out)** and **LQR state feedback**.

---

## Model (Lateral linear model)

This project treats the B-747 lateral linear model in the following form.

$$
\dot{x} = A x + B u,\qquad y = Cx + Du
$$

- State vector (example):  
$$
x = \begin{bmatrix}\beta & r & p & \phi\end{bmatrix}^T
$$
- Input (example):  
$$
u = \begin{bmatrix}\delta_r & \delta_a\end{bmatrix}^T
$$
- Output (example):  
$$
y = \begin{bmatrix}r & \phi\end{bmatrix}^T
$$

> The matrices \(A,B,C,D\) and detailed parameters are included in the scripts/models under `matlab/`.

---

## Controller configuration

### 1) Classical control (Root Locus-based tuning)
A lead/lag compensator was tuned in a SISO loop, and a **wash-out filter** was applied to the yaw damper.

- Compensator (concept):
$$
C(s)=K\frac{s+z}{s+p}\quad (\text{lead/lag})
$$

- Wash-out filter (concept):
$$
W(s)=\frac{s}{s+\omega_w}
\quad\left(\omega_w=\frac{1}{T_w}\right)
$$
It is used to attenuate low-frequency (steady-turn) components while passing the damping-related components needed for stability.

<p align="center">
  <img src="docs/washout_block_diagram.png" alt="Wash-out filter block diagram" width="780">
</p>

---

### 2) LQR state feedback
For the same plant, an LQR-based state-feedback controller was designed to compare performance.

- Control law:
$$
u = -Kx
$$

- Cost function:
$$
J=\int_0^\infty \left(x^TQx + u^TRu\right)\,dt
$$

<p align="center">
  <img src="docs/Simulink%20Implementation%20of%20LQR%20State%20Feedback%20Controller.png" alt="Simulink implementation of LQR state feedback controller" width="780">
</p>

---

## Results

### Rudder doublet response (comparison)
<p align="center">
  <img src="results/lqr/Rudder%20Doublet%20Response.png" alt="Rudder Doublet Response" width="780">
</p>

---

## Repository structure
```text
.
├─ docs/        # Report/illustration images and documents
├─ matlab/      # MATLAB/Simulink design and simulation files
└─ results/     # Simulation result plots (images)
```

---

## How to run (MATLAB/Simulink)

### Requirements
- MATLAB
- Control System Toolbox
- Simulink

## References
- MathWorks (Yaw Damper Design for a 747 Jet Aircraft)  
  https://kr.mathworks.com/help/control/ug/yaw-damper-design-for-a-747-jet-aircraft.html
