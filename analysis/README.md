# Analysis: kinematics, workspace, dynamics and design calculations

This folder explains how the AgroBot arm moves, what it can reach and how strong its motors must be. It has two parts:

- [`mathematica/`](mathematica/): the original symbolic derivation from the master's thesis (22 notebooks, with plain-text copies).
- [`python/`](python/): short, commented scripts that recompute the results for the real robot and produce the figures below.

```bash
pip install -r requirements.txt                  # numpy, sympy, matplotlib, pyyaml; no ROS needed
cd analysis/python
python3 01_kinematics_thesis_model.py            # symbolic kinematics (seconds)
python3 02_workspace.py                          # workspace and approach maps (~2 min)
python3 03_thesis_dynamics_check.py              # check of the thesis torques
python3 04_torque_sizing.py                      # joint torques vs. servo ratings
python3 05_design_calculations.py                # mass budget, gripper, camera, rail
```

| Script | Question it answers | Mathematica notebooks |
|---|---|---|
| `robot_model.py` | Shared model: forward kinematics, Jacobians, M(q), C(q, q̇), G(q) from the URDF | 01–14 (same method) |
| `01_kinematics_thesis_model.py` | Where is each link, and how do joint velocities move it? | 01–07, 19 |
| `02_workspace.py` | Where can the gripper go, and from which direction? | 18 |
| `03_thesis_dynamics_check.py` | Are the thesis torques right? | 12–16 |
| `04_torque_sizing.py` | Are the servos strong enough? | 15–17 |
| `05_design_calculations.py` | Mass, gripper opening, camera resolution, rail cycle | – |

`data/agrobot.urdf` is the robot description expanded from `agrobot_ws/src/aibomech_agrobot_description/urdf/agrobot.urdf.xacro`, so the scripts use exactly the geometry and masses of the simulated and real robot.

---

## 1. Kinematic structure

The arm has four joints. Two turn about vertical axes, as in a SCARA robot, followed by a pitch joint and a wrist:

| joint | axis | role | range |
|---|---|---|---|
| J1 | vertical | shoulder, swings the arm across the row | −62° … 120° |
| J2 | vertical | elbow in the horizontal plane | −120° … 64° |
| J3 | horizontal | pitches the forearm up and down | −115° … 63° |
| J4 | ⊥ J3 | turns the gripper | −57° … 132° |

A trolley on the greenhouse heating pipes adds a fifth, linear axis along the crop row.

### Forward kinematics

Each joint is a rotation about its axis. The pose of link *i* in the base frame is the product of the fixed joint offsets and the joint rotations:

$$
{}^{0}T_{i}(q) = \prod_{k=1}^{i} T_{\text{origin},k}\; \operatorname{Rot}(\hat{z}_k,\, q_k),
\qquad
\operatorname{Rot}(\hat{z}, q) = I + \sin q\,[\hat{z}]_\times + (1-\cos q)\,[\hat{z}]_\times^2
$$

The second formula is Rodrigues' formula. For the thesis model, the centre of mass of the second link, for example, is

$$
p_{c2} = \begin{bmatrix} a_1 \cos q_1 + \tfrac{a_2}{2} \cos(q_1+q_2) \\ a_1 \sin q_1 + \tfrac{a_2}{2} \sin(q_1+q_2) \\ 0 \end{bmatrix}
$$

### Jacobian

The Jacobian maps joint velocities to the velocity of a point. For revolute joint *k* with axis $\hat{z}_k$ through the point $o_k$:

$$
J_v^{(k)} = \hat{z}_k \times (p - o_k), \qquad J_\omega^{(k)} = \hat{z}_k, \qquad \dot{p} = J_v\,\dot{q}
$$

### Inverse kinematics

For the two horizontal links alone there is a closed form (notebook 19):

$$
\cos q_2 = \frac{x^2 + y^2 - a_1^2 - a_2^2}{2 a_1 a_2}, \qquad
q_2 = \operatorname{atan2}\!\left(\sqrt{1-\cos^2 q_2},\ \cos q_2\right), \qquad
q_1 = \operatorname{atan2}(y, x) - \operatorname{atan2}(a_2 \sin q_2,\ a_1 + a_2 \cos q_2)
$$

The full arm has four joints but the gripper needs six numbers to be placed completely (three for position, three for orientation). The robot software therefore solves position exactly, and uses the one remaining degree of freedom to point the gripper as close as possible to the wanted approach direction $\hat{a}$. This is damped least squares with a null-space task (`aibomech_agrobot_tasks/kinematics.py`):

$$
\Delta q = J_p^{+} e_p + \left(I - J_p^{+} J_p\right) (J_\omega N)^{+} (\hat{z}_{\text{tcp}} \times \hat{a}),
\qquad J^{+} = J^\top (J J^\top + \lambda I)^{-1}
$$

## 2. Workspace

`02_workspace.py` samples 40 000 random joint configurations inside the joint limits and plots every reached gripper position (a Monte Carlo workspace, the numeric version of notebook 18):

![Reachable workspace](figures/workspace_views.png)

- Reach from the J1 axis: up to **0.51 m**.
- Gripper height: **−0.09 m to +0.29 m** relative to the mounting plate.

Reaching a point is not enough; the gripper must also come from the right direction. The approach maps solve the IK on a grid and colour each cell by how far the gripper is from the wanted approach direction:

![Approach quality](figures/approach_maps.png)

- **Top-down grasps** (seedlings, weeds) are accurate (within about 10°, mostly 2°) on a band 0.25–0.32 m from J1, 6–7 cm below the mounting plate. This set the bench and tray heights of the transplanting and weeding scenarios.
- **Horizontal grasps into the row** (hanging fruit) work best when the fruit is to the side of J1 along the row, not straight ahead of it (25–50° error there).

That is why the trolley rail matters: for every crop the software moves the trolley so that the crop lands in one of these good regions, exactly like the external axis of an industrial robot cell.

## 3. Dynamics

The equation of motion follows from the Euler–Lagrange method, as in the thesis:

$$
\tau = M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} + G(q)
$$

| term | formula | meaning |
|---|---|---|
| inertia matrix | $M(q) = \sum_i m_i J_{v_i}^\top J_{v_i} + J_{\omega_i}^\top R_i I_i R_i^\top J_{\omega_i}$ | resistance to acceleration, changes with the pose |
| Christoffel symbols | $c_{ijk} = \tfrac{1}{2}\left(\frac{\partial M_{kj}}{\partial q_i} + \frac{\partial M_{ki}}{\partial q_j} - \frac{\partial M_{ij}}{\partial q_k}\right)$ | how M changes as the arm moves |
| Coriolis/centrifugal | $C_{kj} = \sum_i c_{ijk}\,\dot{q}_i$ | torques from moving joints influencing each other |
| gravity | $G(q) = -\sum_i J_{v_i}^\top m_i\, g$ | torque needed to hold the arm still |

`robot_model.py` implements exactly these formulas numerically from the URDF, and checks itself:

- $G(q)$ equals the gradient of the potential energy.
- $M(q)$ is symmetric and positive definite.
- $\dot{M} - 2C$ is skew-symmetric (energy conservation).

### Check of the thesis torques

`03_thesis_dynamics_check.py` takes M(q) exactly as written in notebook 12b and recomputes the example of notebook 16 (q = 30°, 20°, 90°, −15°; q̇ = 0.5, 0.4, 0.4, 0.4 rad/s):

| joint | notebook 16 | recomputed without ½ | with the standard ½ |
|---|---|---|---|
| τ1 | −22.931 | −22.931 | −11.466 |
| τ2 | −3.131 | −3.131 | −1.566 |
| τ3 | 10.586 | 10.586 | 5.293 |
| τ4 | 3.124 | 3.458 | 1.729 |

(kg·cm²/s², 1 kg·cm²/s² = 10⁻⁴ N·m)

The notebook's values are reproduced exactly when the factor ½ is left out of the Christoffel symbols, so the thesis Coriolis/centrifugal torques are twice too large. The notebooks also omit gravity, and their Jacobians of links 3 and 4 differ from the exact derivatives. See [mathematica/README.md](mathematica/README.md#findings-of-the-review).

## 4. Actuator sizing

`04_torque_sizing.py` evaluates the full equation of motion of the real arm: CAD masses including the servos, a 50 g payload in the gripper, a fast move at the velocity limits.

| joint | gravity, empty | gravity, 50 g payload | fast move | servo rating | safety factor |
|---|---|---|---|---|---|
| J1 | 0.000 | 0.000 | 0.128 | 2.9 N·m | 23× |
| J2 | 0.000 | 0.000 | 0.082 | 2.9 N·m | 35× |
| J3 | 0.322 | 0.455 | 0.410 | 2.9 N·m | 6× |
| J4 | 0.048 | 0.115 | 0.070 | 1.5 N·m | 13× |

![Torque sizing](figures/torque_sizing.png)

- J1 and J2 turn about vertical axes, so gravity does not load them; they only accelerate the arm.
- J3 carries the forearm and gripper against gravity. It is the critical joint and still has a safety factor of 6 with a 30 kg·cm (2.9 N·m) smart servo.
- The dynamic torques at these speeds are small compared with gravity. A lighter servo would do for J1 and J2, but a common part for J1–J3 simplifies spares.

## 5. Design calculations

`05_design_calculations.py` prints the numbers behind the cell design.

| item | calculation | result |
|---|---|---|
| Arm mass | CAD parts ×1.10 (fasteners, cables) + one 60 g servo per joint, combined with the parallel-axis theorem $I = I_c + m(\lVert d\rVert^2 I - d d^\top)$ | 0.49 kg moving, 0.31 kg base |
| Gripper opening | gap = jaw position + 17.4 mm | 27 mm open, grips objects up to about 23 mm |
| Camera coverage | width $= 2\,d \tan(\text{HFOV}/2)$ at d = 0.66 m, 69° HFOV | 0.91 m of row, 1.4 mm per pixel, a strawberry ≈ 13 px across |
| Survey stations | 0.40 m apart | 56 % image overlap between stations |
| Rail cycle | quintic profile, $T = 1.875\,\Delta x / v_{\max}$ | 5 s between stations, 3 m row surveyed in ≈ 0.6 min |
| Motion profile | $s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5$ | zero velocity and acceleration at start and end, peak velocity 1.875 × average |

---

## Other material

[`learning/image_segmentation_tutorial.ipynb`](learning/) is a guided PyTorch exercise on image segmentation (segmentation-models-pytorch, human-segmentation dataset, from an online guided project), kept as a starting point for a learned crop detector. It is incomplete and not part of the robot software.
