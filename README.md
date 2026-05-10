# Reaction-Wheel Inverted Pendulum

A reaction-wheel-actuated pendulum stabilised at its unstable upright equilibrium using a two-phase controller: bang-bang energy shaping for swing-up and a discrete LQR with Luenberger observer for stabilisation. Implemented on an STM32F446RE Nucleo running at 500 Hz using STM32 HAL.

> **Status:** Simulation complete. Hardware build in progress.


**TODO: UPDATE FIRMWARE WITH SIM CONSTANTS**

---

## Demo

*Video coming once hardware build is complete.*

Simulation result (30 s run, $\theta_0 = 0.1$ rad):

![Simulation](images/sim_v9.png)

---

## Why This Problem Is Hard

The system has two degrees of freedom (pendulum angle $\theta$, wheel angle $\varphi$) but only one control input (motor torque $\tau$). This underactuation means a single controller cannot handle the full range of motion:

- **Large angles:** the linearisation breaks down entirely — $\sin\theta \neq \theta$
- **Near upright:** the equilibrium is open-loop unstable (eigenvalues $\pm 5.7$ rad/s)
- **No velocity sensor:** $\dot{\theta}$ is not directly measured; it must be estimated

A single LQR cannot swing the pendulum up from rest. A single energy controller cannot stabilise the upright. The solution requires two controllers and a principled switch between them.

---

## Controller Design

### Phase 1 — Bang-Bang Swing-Up

A bang-bang energy pump drives the pendulum from rest toward the upright equilibrium. The control law applies full motor voltage in the direction that adds mechanical energy:

$$
\tau = \begin{cases} -V_{\text{supply}} \cdot \text{sign}(\dot{\theta}) & \text{if } E < E^* \\ 0 & \text{if } E \geq E^* \end{cases}
$$

where $E^* = mgl$ is the target energy (upright with zero velocity) and the current energy is:

$$E = \tfrac{1}{2}ml^2\dot{\theta}^2 - mgl\cos\theta$$

Torque is cut once sufficient energy has accumulated; the pendulum coasts toward the top where LQR engages. A small initial kick ($0.3 \cdot V_{\text{supply}}$) handles the degenerate case where $\dot{\theta} \approx 0$ at startup.

### Phase 2 — Discrete LQR

Near the upright equilibrium, the nonlinear system is linearised by substituting $\delta = \theta - \pi$ (deviation from upright) and using $\sin(\pi + \delta) \approx -\delta$:

$$\ddot{\delta} \approx \frac{g}{l}\delta - \frac{2}{ml^2}\tau$$

This gives a 2-state controllable system $\mathbf{x} = [\delta,\; \dot{\delta}]^\top$. The LQR minimises:

$$J = \sum \left(\mathbf{x}^\top Q\mathbf{x} + R\tau^2\right)$$

with $Q = \text{diag}(300,\, 10)$, $R = 1$. The plant is discretised via zero-order hold at 500 Hz before solving the discrete algebraic Riccati equation.

**Why not design LQR on the full 3-state system $[\theta,\, \dot{\theta},\, \dot{\varphi}]$?** The controllability matrix of the full system has rank 2 — $\dot{\varphi}$ is uncontrollable from $\tau$. Including it in the LQR produces a spurious gain on an unobservable state.

### Switching Condition

LQR engages when both:
- $|\hat{\delta}| < 0.5$ rad (estimated deviation from upright)
- $|\dot{\hat{\delta}}| < 4.0$ rad/s (estimated angular velocity)

Estimates come from the Luenberger observer, not raw measurements.

### Luenberger Observer

The encoder measures $\theta$ with noise ($\sigma \approx 0.02$ rad). Angular velocity $\dot{\theta}$ is not directly sensed. A discrete Luenberger observer runs at 500 Hz alongside the controller:

$$
\begin{aligned}
\hat{\mathbf{x}}[k+1] &= A_d\hat{\mathbf{x}}[k] + B_d\tau[k] \quad \text{(prediction)}\\
\hat{\mathbf{x}}[k] &= \hat{\mathbf{x}}[k] + L\!\left(\delta_{\text{meas}} - \hat{x}_0[k]\right) \quad \text{(correction)}
\end{aligned}
$$

Observer poles are placed at $z = e^{-40\,\Delta t}$ and $z = e^{-60\,\Delta t}$ — roughly 3–5× faster than the closed-loop poles, following standard observer design rules. This gives the observer gain $L = [0.190,\; 4.562]^\top$.

The observer is what makes the LQR switching condition reliable. Using raw encoder position alone to decide when to switch would cause false triggers from measurement noise.

---

## Hardware

| Part | Qty | Notes |
|------|-----|-------|
| STM32F446RE Nucleo-64 | 1 | Already owned |
| Pololu 9.7:1 Metal Gearmotor 25Dx63L mm HP 12V with 48 CPR Encoder (item #4842) | 1 | 9.68:1 gearbox, 4mm D-shaft output, integrated Hall effect quadrature encoder (464.64 CPR at output shaft) |
| Pololu Universal Aluminum Mounting Hub for 4mm Shaft, M3 Holes (item #1997) | 1 | Attaches reaction wheel to gearmotor output shaft via set screw |
| VNH5019 Motor Driver Carrier (Pololu item #1451) | 1 | Handles up to 12A continuous, recommended for HP 25D series |
| AMT103-V encoder kit | 1 | Quadrature A/B mode, DIP switch position 10 = 2048 PPR. Used at pivot to measure $\theta$. The -V suffix means it ships with all sleeve sizes (2–8 mm) — use the sleeve matching your pivot shaft |
| 12V 2A power supply | 1 | Start at 3V during bring-up |

### Physical Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Pendulum length $l$ | 0.1 m | Pivot to motor shaft center |
| Bob mass $m$ | 0.25 kg | Estimated; update from CAD once built |
| Wheel inertia $I_w$ | 0.0003 kg·m² | From Onshape mass properties |
| Damping $b$ | 0.01 N·m·s/rad | Viscous friction estimate |

### Motor Parameters (Pololu #4842, referred to output shaft)

| Parameter | Value | Source |
|-----------|-------|--------|
| $V_{\text{supply}}$ | 12.0 V | Rated |
| $R_m$ (winding resistance) | 2.4 Ω | $V / I_{\text{stall}} = 12.0 / 5.0$ |
| $K_t$ (torque constant) | 0.0762 N·m/A | $\tau_{\text{stall}} / I_{\text{stall}} = 0.3812 / 5.0$ |
| $K_e$ (back-EMF constant) | 0.0762 V·s/rad | $\approx K_t$ at output shaft |
| $\eta$ (gearbox efficiency) | 0.52 | Datasheet max efficiency |
| Stall torque @ 12V (effective) | 0.198 Nm | $\eta \cdot K_t \cdot V / R_m$ |

**These will be verified on hardware** before final controller tuning. $K_t$ in particular should be measured directly.

---

## Repository Structure

```
├── report.pdf                          Full derivation (Lagrangian, LQR, observer design)
├── simulations/
│   └── Sim_v9_added-discretization.ipynb   Simulation with plots
├── Software/
│   └── pendulum_main.ino               Older simple FOC implementation used as a basis
│   └── pendulum_main_arduino           All components for STM32 HAL abstraction level
└── README.md
```

### Simulation

```bash
pip install numpy scipy matplotlib control
```

### Firmware

Firmware is written in STM32 HAL. The Arduino implementation is left as a starting point reference.

## Firmware Constants

Gains from simulation v9 — to be recomputed once real $m$ and $I_w$ are confirmed from hardware:

```c
const float Kd[2]    = {-3.403565f, -0.605939f};
const float Ad[2][2] = {{1.000196f, 0.002000f}, {0.196213f, 1.000196f}};
const float Bd[2]    = {-0.001600f, -1.600105f};
const float Ld[2]    = {0.190356f,  4.561568f};
const float dt       = 0.0020f;  // 500 Hz
```

---

## Design Decisions

**Why a Luenberger observer rather than a complementary filter?**

A complementary filter on encoder + differentiated position is simple but introduces a tuning parameter (cutoff frequency) with no principled connection to the system dynamics. The Luenberger observer poles are placed explicitly relative to the closed-loop poles, and the gain derivation falls directly out of the same state-space framework used for the LQR. Everything stays consistent — physics and math throughout, no hand-tuned filters.

**Why encode $\theta$ at the pivot rather than use an IMU?**

A base encoder gives exact $\theta$ with no drift, no complementary filter, and no phase lag. The IMU approach (used in many student builds) requires careful filter tuning and introduces delay that degrades observer performance. The AMT103-V at 2048 PPR gives sub-0.2° resolution, well below the observer's assumed noise floor.

---

## Planned Extensions

- [ ] Hardware build and experimental validation
- [ ] Measure motor parameters ($K_t$, $K_e$, $R_m$) directly on hardware
- [ ] Characterise encoder noise floor (connects observer $\sigma_{\text{angle}}$ assumption to real data)
- [ ] Bare-metal STM32 HAL rewrite
- [ ] Swing-up controller refinement
- [ ] Extended Kalman Filter replacing Luenberger observer

---

## References

- [V. Hunter Adams — Reaction Wheel Pendulum Lab (Cornell)](https://vanhunteradams.com/Pico/ReactionWheel/ReactionWheel.html)
- [SimpleFOC Reaction Wheel Reference Project](https://github.com/simplefoc/Arduino-FOC-reaction-wheel-inverted-pendulum)
- [SimpleFOC Library Documentation](https://docs.simplefoc.com/)
- Åström & Furuta, *Swinging up a Pendulum by Energy Control* (1996)