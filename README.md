# 🚁 Genetic Algorithm-Based Non-singular Fast Terminal Sliding Mode Control of a Quadrotor with Thrust and Mechanical Link Deflection Fault

This repository presents the design and implementation of an **Optimal Fault-Tolerant Terminal Sliding Mode Control** for a quadrotor UAV subject to **Structural faults**, specifically **rotor thrust and mechanical link deviation**.

The control scheme integrates a **Genetic Algorithm (GA)** optimization and **Radial-Basis-Function Neural Networks (RBFNN)** to optimize the switching and fast controller parameters, to reduce overshoots, to minimuize control efforts, and to estimate the time-varying behavior of fault terms. Comparative simulation studies with a **Nonsingular Fast Terminal Sliding Mode Control (NFTSMC)** and **Disturbance-Observer-Based TSMC** are provided, as wel.

---------------------------------------

## ⚙️ Fault Description and Dynamics

During flight or maneuvering, a drone may inevitably experience structural damage due to collisions with rigid objects, such as another UAV, a tree, or even an obstacle. Such incidents may cause one of the thrust forces of the drone to no longer operate in a vertical direction. It is important to emphasize that this collision does not necessarily degrade the motor efficiency but merely alters its orientation. The angles of deviation caused by such collision are demonstrated in Figure.
The Assumptions regarding the mathematical modeling are described as follows:

**Assumption 1.** The position of the center of mass is constant after the fault occurrence.
**Assumption 2.** The symmetry of the moment of inertia will not change after the fault occurrence.
**Assumption 3.** After the fault occurrence, the moment of inertia and the mass value of the drone will not change.

The deflections from vertical directions add additional nonlinear terms to the system dynamics mathematically expressed as:

$$
\begin{aligned}
\ddot{x} &= (s_{\phi}s_{\psi} + c_{\phi} s_{\theta} c_{\psi}) \frac{u_T}{m} - \frac{K_f}{m} \dot{x} + f_{stx} \\
\ddot{y} &= (-s_{\phi} c_{\psi} + c_{\phi} s_{\theta} s_{\psi}) \frac{u_T}{m} - \frac{K_f}{m} \dot{y} + f_{sty} \\
\ddot{z} &= -g + (c_{\phi} c_{\theta}) \frac{u_T}{m} - \frac{K_f}{m} \dot{z} + f_{stz} \\
\ddot{\phi} &= \left(\frac{I_y - I_z}{I_x}\right) \dot{\theta} \dot{\psi} + \frac{J_{TP}}{I_x} \dot{\theta} \omega^* + \frac{u_\phi}{I_x} - \frac{K_tL}{I_x} \dot{\phi} + f_{st\phi} \\
\ddot{\theta} &= \left(\frac{I_z - I_x}{I_y}\right) \dot{\phi} \dot{\psi} - \frac{J_{TP}}{I_y} \dot{\phi} \omega^* + \frac{u_\theta}{I_y} - \frac{K_tL}{I_y} \dot{\theta} + f_{st\theta} \\
\ddot{\psi} &= \left(\frac{I_x - I_y}{I_z}\right) \dot{\phi} \dot{\theta} + \frac{u_\psi}{I_z} - \frac{K_tL}{I_z} \dot{\psi} + f_{st\psi}
\end{aligned}
$$

where:

$$
\begin{aligned}
f_{stx} &= \frac{b}{m} \omega_2^2 \left( f_1 \cos\theta \cos\psi + f_2 (\cos\psi \sin\phi \sin\theta - \cos\phi \sin\psi) + f_3 (\sin\phi \sin\psi + \cos\phi \cos\psi \sin\theta) \right) \\
f_{sty} &= \frac{b}{m} \omega_2^2 \left( f_1 \cos\theta \sin\psi + f_2 (\sin\psi \sin\phi \sin\theta + \cos\phi \sin\psi) + f_3 (-\sin\phi \cos\psi + \cos\phi \sin\psi \sin\theta) \right) \\
f_{stz} &= \frac{b}{m} \omega_2^2 \left( -f_1 \sin\theta + f_2 (\sin\phi \cos\theta) + f_3 (\cos\phi \cos\theta) \right) \\
f_{st\phi} &= \frac{J_{TP}}{I_x} \omega_2 (f_2 \dot{\psi} - f_3 \dot{\theta}) + \frac{1}{I_x} \omega_2^2 (b l f_4 + f_1 d) \\
f_{st\theta} &= \frac{J_{TP}}{I_y} \omega_2 (f_1 \dot{\psi} + f_3 \dot{\phi}) + \frac{1}{I_y} \omega_2^2 (b l f_5 - f_2 d) \\
f_{st\psi} &= -\frac{J_{TP}}{I_z} \omega_2 (f_2 \dot{\phi} + f_1 \dot{\theta}) - \frac{1}{I_z} \omega_2^2 (b l f_6 + f_3 d)
\end{aligned}
$$

and fault related terms regarding the fault angles $\alpha$, $\beta$, and $\gamma$ are expressed as follows:

$$
\begin{aligned}
f_1 &= \sin\alpha \sin\gamma \\
f_2 &= -\cos\gamma \sin\beta + \sin\gamma \cos\beta \cos\alpha \\
f_3 &= \cos\beta \cos\gamma + \cos\alpha \sin\beta \sin\gamma - 1 \\
f_4 &= f_2 \sin\beta - (1 + f_3) \cos\beta + 1 \\
f_5 &= f_1 \sin\beta \\
f_6 &= f_1 \cos\beta
\end{aligned}
$$

------------------

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
