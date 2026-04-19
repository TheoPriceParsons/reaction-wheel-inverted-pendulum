# Reaction-Wheel Inverted Pendulum

A reaction-wheel-actuated pendulum stabilised at its unstable upright equilibrium using a two-phase controller: bang-bang energy shaping for swing-up and a discrete LQR with Luenberger observer for stabilisation. Implemented on an STM32F446RE Nucleo running at 500 Hz via SimpleFOC.

> **Status:** Simulation complete. Hardware build in progress.

---

## Demo

*Video coming once hardware build is complete.*

Simulation result (30 s run, θ₀ = 0.1 rad):

![Simulation](pendulum_v9.png)

---

## Why This Problem Is Hard

The system has two degrees of freedom (pendulum angle θ, wheel angle φ) but only one control input (motor torque τ). This underactuation means a single controller cannot handle the full range of motion:

- **Large angles:** the linearisation breaks down entirely — sin θ ≠ θ
- **Near upright:** the equilibrium is open-loop unstable (eigenvalues ±5.7 rad/s)
- **No velocity sensor:** θ̇ is not directly measured; it must be estimated

A single LQR cannot swing the pendulum up from rest. A single energy controller cannot stabilise the upright. The solution requires two controllers and a principled switch between them.

---

## Controller Design

### Phase 1 — Bang-Bang Swing-Up

A bang-bang energy pump drives the pendulum from rest toward the upright equilibrium. The control law applies full motor voltage in the direction that adds mechanical energy:

```
τ = −V_supply · sign(θ̇)    if E < E*
τ = 0                        if E ≥ E*
```

where `E* = mgl` is the target energy (upright with zero velocity) and the current energy is:

```
E = ½ml²θ̇² − mgl·cos θ
```

Torque is cut once sufficient energy has accumulated; the pendulum coasts toward the top where LQR engages. A small initial kick (`0.3 · V_supply`) handles the degenerate case where θ̇ ≈ 0 at startup.

### Phase 2 — Discrete LQR

Near the upright equilibrium, the nonlinear system is linearised by substituting δ = θ − π (deviation from upright) and using sin(π + δ) ≈ −δ:

```
δ̈ ≈ (g/l)·δ − (2/ml²)·τ
```

This gives a 2-state controllable system `x = [δ, δ̇]ᵀ`. The LQR minimises:

```
J = Σ (xᵀQx + Rτ²)
```

with `Q = diag(20, 2)`, `R = 1`. The plant is discretised via zero-order hold at 500 Hz before solving the discrete algebraic Riccati equation.

**Why not design LQR on the full 3-state system [θ, θ̇, φ̇]?** The controllability matrix of the full system has rank 2 — φ̇ is uncontrollable from τ. Including it in the LQR produces a spurious gain on an unobservable state.

### Switching Condition

LQR engages when both:
- `|δ̂| < 0.5 rad` (estimated deviation from upright)
- `|δ̇̂| < 4.0 rad/s` (estimated angular velocity)

Estimates come from the Luenberger observer, not raw measurements.

### Luenberger Observer

The encoder measures θ with noise (σ ≈ 0.02 rad). Angular velocity θ̇ is not directly sensed. A discrete Luenberger observer runs at 500 Hz alongside the controller:

```
x̂[k+1] = Ad·x̂[k] + Bd·τ[k]          (prediction)
x̂[k]   = x̂[k] + L·(δ_meas − x̂₀[k]) (correction)
```

Observer poles are placed at z = exp(−40·dt) and z = exp(−60·dt) — roughly 3–5× faster than the closed-loop poles, following standard observer design rules. This gives the observer gain `L = [0.190, 4.419]ᵀ`.

The observer is what makes the LQR switching condition reliable. Using raw encoder position alone to decide when to switch would cause false triggers from measurement noise.

---

## Hardware

| Component | Part | Notes |
|-----------|------|-------|
| Microcontroller | STM32F446RE Nucleo-64 | 180 MHz Cortex-M4, hardware FPU |
| Motor driver | MKS Dual FOC V3.2 | Used as 3-phase H-bridge (3-PWM mode) |
| Motor | iFlight GM4108H-120T | 120KV brushless gimbal motor, 8mm hollow shaft |
| Encoders | CUI AMT103-V × 2 | 2048 CPR, one on motor shaft, one on pivot |
| Power | 3S LiPo 11.1V ≥ 1000mAh | |

### Wiring

The MKS board is designed for ESP32 but the 3-phase outputs are wired directly to PWM-capable pins on the Nucleo. SimpleFOC handles commutation in 3-PWM voltage mode.

```
Nucleo PA8  →  MKS A0
Nucleo PA9  →  MKS B0
Nucleo PA10 →  MKS C0
Common ground
```

```cpp
BLDCDriver3PWM driver(PA8, PA9, PA10);
```

Both AMT103 encoders connect directly to Nucleo GPIO pins in quadrature mode.

### Motor Parameters

The GM4108H-120T is a gimbal motor — low KV, low winding resistance, designed for smooth torque at low speeds. Approximate parameters used in simulation:

| Parameter | Value | Source |
|-----------|-------|--------|
| K_t (torque constant) | 0.03 Nm/A | Estimated |
| K_e (back-EMF constant) | 0.00954 V·s/rad | Estimated |
| R_m (winding resistance) | 2.4 Ω | Estimated |
| Stall torque @ 12V | ~0.15 Nm | Derived |

**These will be measured on hardware** before tuning the controller. The observer and LQR gains depend on K_t being accurate.

---

## Repository Structure

```
├── report.pdf                          Full derivation (Lagrangian, LQR, observer design)
├── simulations/
│   └── Sim_v9_added-discretization.ipynb   Simulation with plots
├── Software/
│   └── pendulum_main/
│       └── pendulum_main.ino           STM32 firmware (SimpleFOC)
└── README.md
```

### Simulation

```bash
pip install numpy scipy matplotlib control
jupyter notebook simulations/Sim_v9_added-discretization.ipynb
```

### Firmware

Requires Arduino IDE 2.x with STM32duino and SimpleFOC libraries installed. Open `Software/pendulum_main/pendulum_main.ino` and flash to Nucleo.

**Note:** `V_SUPPLY` is currently set to `3.0f` for bench testing without a load. Change to `12.0f` for full operation.

---

## Firmware Constants

The discretisation and LQR/observer gains are computed in simulation and copied directly into firmware:

```c
const float Kd[2]    = {-4.976694f, -1.404655f};
const float Ad[2][2] = {{1.000065f, 0.002000f}, {0.065401f, 1.000065f}};
const float Bd[2]    = {-0.000089f, -0.088891f};
const float Ld[2]    = {0.190094f,  4.418506f};
const float dt       = 0.0020f;  // 500 Hz
```

---

## Design Decisions

**Why SimpleFOC instead of bare-metal STM32 HAL?**
The interesting engineering problem here is the controls design — the Lagrangian derivation, LQR discretisation, and observer. Reimplementing a motor driver from scratch would have taken months and shifted focus away from that. SimpleFOC abstracts the commutation and lets the controller be the subject. A bare-metal STM32 implementation is a planned follow-on project.

**Why a gimbal motor?**
The GM4108H has a hollow shaft, which simplifies the pendulum pivot and bearing design. Low KV also means higher torque at low RPM, which is what the swing-up phase needs. The tradeoff is higher winding inductance and sensitivity to back-EMF at speed.

**Why a Luenberger observer rather than a complementary filter?**
A complementary filter on encoder + differentiated position is simple but introduces a tuning parameter (cutoff frequency) with no principled connection to the system dynamics. The Luenberger observer poles are placed explicitly relative to the closed-loop poles, and the gain derivation falls directly out of the same state-space framework used for the LQR. Everything stays consistent.

---

## Planned Extensions

- [ ] Hardware build and experimental validation
- [ ] Measure motor parameters (K_t, K_e, R_m) directly
- [ ] Characterise encoder noise floor (connects observer sigma_angle assumption to real data)
- [ ] Bare-metal STM32 HAL rewrite with custom FOC implementation
- [ ] Extended Kalman Filter replacing Luenberger observer

---

## References

- [SimpleFOC Reaction Wheel Reference Project](https://github.com/simplefoc/Arduino-FOC-reaction-wheel-inverted-pendulum)
- [SimpleFOC Library Documentation](https://docs.simplefoc.com/)
- Åström & Furuta, *Swinging up a Pendulum by Energy Control* (1996)
