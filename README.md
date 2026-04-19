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


## Wiring 

Since we weren't able to get the right FOC shield we need to reuse one designed for ESP32 and connect the A0, B0, C0 of the motor driver to any PWM capable pins on the Nucleo-F446re: 
'''
BLDCDriver3PWM driver(PA8, PA9, PA10);// A0->PA8 || B0->PA9  || C0->PA10
'''
We make these connections thru jumper wires.
