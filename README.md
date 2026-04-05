# Reaction-Wheel Inverted Pendulum

Simulation of a reaction-wheel-actuated pendulum with a two-phase controller: bang-bang energy pump for swing-up, LQR for upright stabilisation. Full derivation in `report.pdf`.

## How it works

`θ = 0` is down, `θ = π` is up (target).

**Swing-up** — bang-bang energy pump applies full torque in whichever direction adds energy:

$τ = u_max · sign(θ̇ )$   if $E < E*$




$τ = 0$                                if $E ≥ E*$


**Stabilisation** — LQR engages once within `|δ| < 0.3 rad` and `|δ̇| < 2.0 rad/s` of upright.

## Requirements
```
pip install numpy scipy matplotlib control
```

## Usage
```
python pendulum_v4.py
```

## Files

| File | Description |
|------|-------------|
| `pendulum_v4.py` | simulation and controller |
| `report.pdf` | full derivation — Lagrangian mechanics, LQR design |
