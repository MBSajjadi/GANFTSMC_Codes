# Genetic Algorithm-Based Non-singular Fast Terminal Sliding Mode Control of a Quadrotor with Thrust and Mechanical Link Deflection Fault

This repository presents the design and implementation of an **Optimal Fault-Tolerant Terminal Sliding Mode Control** for a quadrotor UAV subject to **Structural faults**, specifically **rotor thrust and mechanical link deviation**.

The control scheme integrates a **Genetic Algorithm (GA)** optimization and **Radial-Basis-Function Neural Networks (RBFNN)** to optimize the switching and fast controller parameters, to reduce overshoots, to minimuize control efforts, and to estimate the time-varying behavior of fault terms. Comparative simulation studies with a **Nonsingular Fast Terminal Sliding Mode Control (NFTSMC)** and **Disturbance-Observer-Based TSMC** are provided, as wel.

---------------------------------------

## 1. System Modeling

### 1.1 Quadrotor Dynamics

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

### 1.2 Fault Modeling

A **rotor deviation fault** is modeled as:

$$
u_i = (1 - \delta_i) u_i^*
$$

where:

* $\delta_i$ = deviation factor of rotor $i$,
* $u_i^*$ = nominal thrust of rotor $i$.

This represents **thrust loss** or **structural misalignment**.

---

## 2. Terminal Sliding Mode Control (TSMC)

### 2.1 Sliding Surface

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

### 2.2 Control Law

The TSMC control input is designed as:

$$
u_i = u_{eq,i} - K_i , \text{sign}(s_i)
$$

* $u_{eq,i}$ = equivalent control,
* $K_i > 0$ = switching gain ensuring robustness.

---

## 3. Optimization with Genetic Algorithm (GA)

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

## 4. Simulation Codes

The repository provides several simulation files:

* **Main_RBFFNN_GANFTSMC.m**
  Runs the optimized **RBFNN + GA-TSMC** controller.
  Outputs system states and **3D trajectory** of the faulty quadrotor.

* **PlotComparedResults_GANFTSMC_NFTSMC.m**
  Compares **GANFTSMC** with **standard NFTSMC**.

* **Final_SecondTrajectory.m**
  Shows results of **Disturbance-Observer-Based TSMC** for additional comparison.

---

## 5. Results

### 5.1 Trajectory Tracking

* GANFTSMC achieves accurate path tracking even under rotor deviation.
* NFTSMC shows larger overshoots.

### 5.2 Control Efforts

* GANFTSMC reduces input magnitudes while maintaining robustness.

### 5.3 Fault Tolerance

* Tracking errors converge to zero in finite time despite **structural faults**.

(Plots and figures are included in the `figures/` folder.)

---

## 6. Repository Structure

```
├── src/                        # MATLAB or Python source files
├── simulations/                # Simulation scripts
├── figures/                    # Plots and figures
├── README.md                   # Project documentation
```

---

## 7. How to Use

1. Clone this repository:

   ```bash
   git clone https://github.com/YourUsername/Quadrotor-TSMC-Fault-Tolerant.git
   ```

2. Open MATLAB.

3. Run the following main scripts:

   * `Main_RBFFNN_GANFTSMC.m` → optimized controller simulation.
   * `PlotComparedResults_GANFTSMC_NFTSMC.m` → comparison plots.
   * `Final_SecondTrajectory.m` → disturbance observer-based TSMC.

----------------
