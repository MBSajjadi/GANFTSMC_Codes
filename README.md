# Genetic Algorithm-Based Non-singular Fast Terminal Sliding Mode Control of a Quadrotor with Thrust and Mechanical Link Deflection Fault

This repository presents the design and implementation of a **Fault-Tolerant Terminal Sliding Mode Control (TSMC)** for a quadrotor UAV subject to **structural anomalies**, specifically **rotor thrust deviation faults**.

The control scheme integrates a **Genetic Algorithm (GA)** to optimize the controller parameters, reducing overshoot and minimizing control efforts. Additionally, comparisons with standard **Nonsingular Fast Terminal Sliding Mode Control (NFTSMC)** and **Disturbance-Observer-Based TSMC** are provided.

--------------------------

## 1. Introduction

Quadrotors are widely used in industrial, surveillance, and autonomous operations. However, they are vulnerable to **structural anomalies** such as rotor deviations, which may degrade performance and stability.

This project introduces a **robust control strategy** based on **Terminal Sliding Mode Control (TSMC)** and optimizes its gains using a **Genetic Algorithm** to ensure accurate tracking under faulty conditions.

---

## 2. System Modeling

### 2.1 Quadrotor Dynamics

The nonlinear dynamics of the quadrotor are described as:

$$
m \ddot{x} = -u_1 (\cos\phi \sin\theta \cos\psi + \sin\phi \sin\psi)
$$

$$
m \ddot{y} = -u_1 (\cos\phi \sin\theta \sin\psi - \sin\phi \cos\psi)
$$

$$
m \ddot{z} = u_1 \cos\phi \cos\theta - mg
$$

where:

* $x, y, z$ → translational states,
* $\phi, \theta, \psi$ → roll, pitch, and yaw angles,
* $u_1$ → thrust input,
* $m, g$ → mass and gravitational constant.

---

### 2.2 Fault Modeling

A **rotor deviation fault** is modeled as:

$$
u_i = (1 - \delta_i) u_i^*
$$

where:

* $\delta_i$ = deviation factor of rotor $i$,
* $u_i^*$ = nominal thrust of rotor $i$.

This represents **thrust loss** or **structural misalignment**.

---

## 3. Terminal Sliding Mode Control (TSMC)

### 3.1 Sliding Surface

Tracking error:

$$
e_i = x_i - x_{i,d}, \quad i \in {x,y,z,\phi,\theta,\psi}
$$

Terminal sliding surface:

$$
s_i = e_i + c_i |e_i|^{\alpha_i} ,\text{sign}(e_i)
$$

with $0 < \alpha_i < 1$ ensuring **finite-time convergence**.

---

### 3.2 Control Law

The TSMC control input is designed as:

$$
u_i = u_{eq,i} - K_i , \text{sign}(s_i)
$$

* $u_{eq,i}$ = equivalent control,
* $K_i > 0$ = switching gain ensuring robustness.

---

## 4. Optimization with Genetic Algorithm (GA)

To reduce overshoots and control efforts, the **Genetic Algorithm** optimizes controller parameters $\eta$ and $K$.

* **Cost Function:** weighted sum of tracking error and control effort.
* **Simulation Time:** 50 seconds.
* **Sampling Time:** $T_s = 0.001$ s.
* **Process:**

  1. GA generates a population of candidate parameters.
  2. Each candidate is evaluated by running the simulation.
  3. Higher-cost candidates are eliminated.
  4. Crossover and mutation generate new parameters.
  5. Optimal $\eta$ and $K$ are selected after convergence.

---

## 5. Simulation Codes

The repository provides several simulation files:

* **Main_RBFFNN_GANFTSMC.m**
  Runs the optimized **RBFNN + GA-TSMC** controller.
  Outputs system states and **3D trajectory** of the faulty quadrotor.

* **PlotComparedResults_GANFTSMC_NFTSMC.m**
  Compares **GANFTSMC** with **standard NFTSMC**.

* **Final_SecondTrajectory.m**
  Shows results of **Disturbance-Observer-Based TSMC** for additional comparison.

---

## 6. Results

### 6.1 Trajectory Tracking

* GANFTSMC achieves accurate path tracking even under rotor deviation.
* NFTSMC shows larger overshoots.

### 6.2 Control Efforts

* GANFTSMC reduces input magnitudes while maintaining robustness.

### 6.3 Fault Tolerance

* Tracking errors converge to zero in finite time despite **structural faults**.

(Plots and figures are included in the `figures/` folder.)

---

## 7. Repository Structure

```
├── src/                        # MATLAB or Python source files
├── simulations/                # Simulation scripts
├── figures/                    # Plots and figures
├── README.md                   # Project documentation
```

---

## 8. How to Use

1. Clone this repository:

   ```bash
   git clone https://github.com/YourUsername/Quadrotor-TSMC-Fault-Tolerant.git
   ```

2. Open MATLAB.

3. Run the following main scripts:

   * `Main_RBFFNN_GANFTSMC.m` → optimized controller simulation.
   * `PlotComparedResults_GANFTSMC_NFTSMC.m` → comparison plots.
   * `Final_SecondTrajectory.m` → disturbance observer-based TSMC.

---

## 9. References

* Utkin, V. I. *Sliding Modes in Control and Optimization*. Springer, 1992.
* Edwards, C., Spurgeon, S. K. *Sliding Mode Control: Theory and Applications*. Taylor & Francis, 1998.
* Recent works on Fault-Tolerant Control of UAVs.

---

## 10. License

This project is licensed under the MIT License.
