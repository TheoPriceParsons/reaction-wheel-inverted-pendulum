# Reaction-Wheel Inverted Pendulum

Simulation of a reaction-wheel-actuated pendulum with a two-phase controller: bang-bang energy pump for swing-up, LQR for upright stabilisation. Full derivation in `report.pdf`.

## How it works

**Convention:** `θ = 0` is down, `θ = π` is up (control target).

**Swing-up** — bang-bang energy pump applies full torque whenever the pendulum needs energy, in the direction that adds it:
```
τ = u_max · sign(θ̇ · cos(θ − π))   if E < E*
τ = 0                                 if E ≥ E*
```

**Stabilisation** — LQR engages once the pendulum is within `|δ| < 0.3 rad` and `|δ̇| < 2.0 rad/s` of upright.

## Requirements

```
numpy scipy matplotlib python-control
```

Install with:
```
pip install numpy scipy matplotlib control
```

## Usage

```
python pendulum_v4.py
```

Produces two plots and prints a final-state report to the terminal.

## Files

```
pendulum_v4.py   simulation and controller
report.pdf       full derivation (Lagrangian mechanics, LQR design)
```

## Parameters

| Symbol | Value | Description |
|--------|-------|-------------|
| m      | 0.5 kg | Pendulum bob mass |
| l      | 0.3 m  | Pendulum length |
| I_w    | 0.002 kg·m² | Wheel inertia |
| u_max  | 0.5 Nm | Torque saturation |


