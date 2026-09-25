# Kinematics of the AgroBot arm

This page derives where the gripper is for given joint angles (forward kinematics), how fast it moves (Jacobian) and which joint angles reach a given point (inverse kinematics). It follows the published analysis of the arm, rebuilt step by step, with a worked numeric example that you can reproduce with [`python/01_kinematics_thesis_model.py`](python/01_kinematics_thesis_model.py).

> B. Altawil and F. C. Can, "Design and Analysis of a Four DoF Robotic Arm with Two Grippers Used in Agricultural Operations," *International Journal of Applied Mathematics Electronics and Computers*, 11(2), 79–87, 2023. [doi:10.18100/ijamec.1217072](https://doi.org/10.18100/ijamec.1217072). A text copy is in [`paper/`](paper/altawil2023_four_dof_agricultural_arm.md).

**Contents:** [1. The arm](#1-the-arm) · [2. The Denavit–Hartenberg method](#2-the-denavithartenberg-method) · [3. D-H table](#3-d-h-table-of-the-arm) · [4. Transformation matrices](#4-transformation-matrices) · [5. Forward kinematics of both end effectors](#5-forward-kinematics-of-both-end-effectors) · [6. Worked example](#6-worked-example) · [7. Jacobian](#7-velocity-kinematics-the-jacobian) · [8. Inverse kinematics](#8-inverse-kinematics) · [9. From the paper model to the ROS 2 robot](#9-from-the-paper-model-to-the-ros-2-robot)

## 1. The arm

![The 3D-printed prototype](../docs/images/real_robot.jpeg)

The arm has four revolute joints:

| Joint | Axis | Role |
|---|---|---|
| J1 | vertical | shoulder: swings the arm in the horizontal plane |
| J2 | vertical | elbow: together with J1 forms a SCARA pair |
| J3 | horizontal | pitches the forearm down towards the plants |
| J4 | wrist | orients the end effector |

J1 and J2 move in the horizontal plane, so gravity does not act on them; this makes a SCARA pair light and fast. J3 lifts and lowers the forearm and carries the weight of everything after it.

The paper's arm carries **two end effectors** on the wrist: a nozzle fed by a peristaltic pump for fertilising (end effector 1, **e1**) and a servo gripper for harvesting (end effector 2, **e2**).

## 2. The Denavit–Hartenberg method

The Denavit–Hartenberg (D-H) convention describes a serial robot with four numbers per joint. Every joint *i* gets a coordinate frame, placed by three rules:

1. $z_{i-1}$ is the axis of joint *i*.
2. $x_i$ lies along the common normal from $z_{i-1}$ to $z_i$.
3. $y_i$ completes a right-handed frame.

Frame *i* then follows from frame *i−1* by four elementary motions:

| Parameter | Motion | Meaning |
|---|---|---|
| $\theta_i$ | rotation about $z_{i-1}$ | joint angle (the variable of a revolute joint) |
| $d_i$ | translation along $z_{i-1}$ | offset along the joint axis |
| $a_i$ | translation along $x_i$ | link length |
| $\alpha_i$ | rotation about $x_i$ | twist between successive joint axes |

Multiplying the four motions gives the homogeneous transformation of one link:

```math

A_i
= \mathrm{Rot}_z(\theta_i)\,\mathrm{Trans}_z(d_i)\,
  \mathrm{Trans}_x(a_i)\,\mathrm{Rot}_x(\alpha_i)
= \begin{bmatrix}
C_{\theta_i} & -S_{\theta_i} C_{\alpha_i} & S_{\theta_i} S_{\alpha_i} & a_i C_{\theta_i} \\
S_{\theta_i} & C_{\theta_i} C_{\alpha_i} & -C_{\theta_i} S_{\alpha_i} & a_i S_{\theta_i} \\
0 & S_{\alpha_i} & C_{\alpha_i} & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}

```

$C$ and $S$ abbreviate cosine and sine, and sums of indices mean sums of angles: $C_{12} = \cos(\theta_1 + \theta_2)$, $S_{34} = \sin(\theta_3 - \theta_4)$ (the paper defines $\theta_{34}$ as the difference). The upper-left 3×3 block is the rotation of the frame, the last column its position.

The pose of frame *n* in the base frame is the product along the chain:

```math
{}^{0}T_{n} = A_1 A_2 \cdots A_n
```

## 3. D-H table of the arm

This is Table 1 of the paper, with the lengths of the thesis model:

| Link *i* | $a_i$ | $\alpha_i$ | $d_i$ | $\theta_i$ | Range |
|---|---|---|---|---|---|
| 1 | $a_1$ = 12 cm | 0 | 0 | $\theta_1$ | 0–170° |
| 2 | $a_2$ = 12 cm | $-\pi/2$ | 0 | $\theta_2$ | 0–170° |
| 3 | $a_3$ = 8 cm | $-\pi/2$ | 0 | $\theta_3$ | 0–170° |
| 4 | 0 | $-\pi/2$ | $d_4$ = 8 cm | 0 | fixed |
| 4e1 (nozzle) | 0 | 0 | $d_{4e1}$ | $\theta_4$ | 0–170° |
| 4e2 (gripper) | $a_{4e2}$ = 8.5 cm | 0 | 0 | $\theta_4$ | 0–170° |

- $\alpha_1 = 0$ keeps the axes of J1 and J2 parallel, both vertical (the SCARA pair).
- $\alpha_2 = -\pi/2$ turns the next axis horizontal (J3).
- Row 4 is a fixed frame at the wrist; rows 4e1 and 4e2 are the two end effectors, both turned by $\theta_4$.
- The paper gives no value for $d_{4e1}$; the worked example assumes 3 cm.
- The notebooks call $d_4$ "$d_3$" and $a_{4e2}$ "$a_4$".

## 4. Transformation matrices

**Joint 1 to the base (equation 3):**

```math
{}^{0}T_{1} = \begin{bmatrix}
C_1 & -S_1 & 0 & a_1 C_1 \\
S_1 & C_1 & 0 & a_1 S_1 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

**Joint 2 to the base (equation 4).** The two vertical joints add up to $\theta_{12}$:

```math
{}^{0}T_{2} = \begin{bmatrix}
C_{12} & 0 & -S_{12} & a_1 C_1 + a_2 C_{12} \\
S_{12} & 0 & C_{12} & a_1 S_1 + a_2 S_{12} \\
0 & -1 & 0 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

**Joint 3 to the base (equation 5).** To keep the matrices short, the paper introduces (equation group 1)

```math
\beta = C_{12} C_3, \qquad \gamma = C_{12} S_3, \qquad \delta = S_{12} C_3, \qquad \varepsilon = S_{12} S_3
```

```math
{}^{0}T_{3} = \begin{bmatrix}
\beta & S_{12} & -\gamma & a_1 C_1 + a_2 C_{12} + a_3 \beta \\
\delta & -C_{12} & -\varepsilon & a_1 S_1 + a_2 S_{12} + a_3 \delta \\
-S_3 & 0 & -C_3 & -a_3 S_3 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

**The wrist frame (equation 6):**

```math
{}^{0}T_{4} = \begin{bmatrix}
\beta & \gamma & S_{12} & a_1 C_1 + a_2 C_{12} + a_3 \beta - d_4 \gamma \\
\delta & \varepsilon & -C_{12} & a_1 S_1 + a_2 S_{12} + a_3 \delta - d_4 \varepsilon \\
-S_3 & C_3 & 0 & -a_3 S_3 - d_4 C_3 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

The third column is the wrist axis $z_4 = (S_{12}, -C_{12}, 0)$. It is horizontal and parallel to the axis of J3, so in this model J3 and J4 both pitch the tool and their angles combine as $\theta_{34} = \theta_3 - \theta_4$.

**The end effectors (equations 7 and 8).** With the paper's abbreviation $\forall = C_{12} C_{34}$ (equation group 2), both end effectors share the rotation

```math
{}^{0}R_{e} = \begin{bmatrix}
\forall & C_{12} S_{34} & S_{12} \\
S_{12} C_{34} & S_{12} S_{34} & -C_{12} \\
-S_{34} & C_{34} & 0
\end{bmatrix}
```

and differ only in their position, given in the next section.

## 5. Forward kinematics of both end effectors

The position column of ${}^{0}T_{e1}$ and ${}^{0}T_{e2}$ gives the forward kinematics.

**End effector 1, the nozzle (equation group 3).** It sits $d_{4e1}$ along the wrist axis:

```math
\begin{aligned}
P_{e1x} &= a_1 C_1 + a_2 C_{12} + a_3 \beta - d_4 \gamma + d_{4e1} S_{12} \\
P_{e1y} &= a_1 S_1 + a_2 S_{12} + a_3 \delta - d_4 \varepsilon - d_{4e1} C_{12} \\
P_{e1z} &= -a_3 S_3 - d_4 C_3
\end{aligned}
```

**End effector 2, the gripper (equation group 4).** It sits $a_{4e2}$ along the wrist's $x$ axis, so it turns with $\theta_4$:

```math
\begin{aligned}
P_{e2x} &= a_1 C_1 + a_2 C_{12} + a_3 \beta + a_{4e2} \forall - d_4 \gamma \\
P_{e2y} &= a_1 S_1 + a_2 S_{12} + a_3 \delta + a_{4e2} S_{12} C_{34} - d_4 \varepsilon \\
P_{e2z} &= -a_3 S_3 - d_4 C_3 - a_{4e2} S_{34}
\end{aligned}
```

The script multiplies the D-H matrices symbolically and confirms that both results match these expressions. In the converted text of the paper, $P_{e2x}$ reads $a_{4e2}\beta$ where the D-H product gives $a_{4e2}\forall$; the abbreviation $\forall$ is defined for exactly this term.

**Reading the equations.**

- $a_1 C_1 + a_2 C_{12}$ is the classic two-link SCARA position in the horizontal plane.
- The terms with $\beta, \gamma, \delta, \varepsilon$ add the forearm, which points along the direction $\theta_{12}$ and is tilted by $\theta_3$.
- The height depends only on $\theta_3$ (and $\theta_4$ for the gripper), never on $\theta_1, \theta_2$, because J1 and J2 are vertical.

## 6. Worked example

Take $\theta = (30^\circ, 20^\circ, 90^\circ, -15^\circ)$, the pose also used for the torques in [dynamics.md](dynamics.md#7-worked-example), with $a_1 = a_2 = 12$, $a_3 = d_4 = 8$, $a_{4e2} = 8.5$ and (assumed) $d_{4e1} = 3$ cm.

**By hand, joint 2:** $a_1 C_1 + a_2 C_{12} = 12\cos 30^\circ + 12 \cos 50^\circ = 10.392 + 7.713 = 18.106$ cm, and $a_1 S_1 + a_2 S_{12} = 6.000 + 9.193 = 15.193$ cm.

**By hand, the wrist.** With $\theta_3 = 90^\circ$: $\beta = \delta = 0$, $\gamma = C_{50^\circ} = 0.643$, $\varepsilon = S_{50^\circ} = 0.766$, so

```math
{}^{0}p_{4} = \begin{bmatrix} 18.106 - 8 \cdot 0.643 \\ 15.193 - 8 \cdot 0.766 \\ -8 \cdot 1 - 8 \cdot 0 \end{bmatrix}
= \begin{bmatrix} 12.964 \\ 9.064 \\ -8.000 \end{bmatrix} \text{ cm}
```

The full matrices printed by the script:

```math
{}^{0}T_{2} = \begin{bmatrix}
0.643 & 0 & -0.766 & 18.106 \\
0.766 & 0 & 0.643 & 15.193 \\
0 & -1 & 0 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
{}^{0}T_{4} = \begin{bmatrix}
0 & 0.643 & 0.766 & 12.964 \\
0 & 0.766 & -0.643 & 9.064 \\
-1 & 0 & 0 & -8.000 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

```math
{}^{0}T_{e2} = \begin{bmatrix}
-0.166 & 0.621 & 0.766 & 11.549 \\
-0.198 & 0.740 & -0.643 & 7.379 \\
-0.966 & -0.259 & 0 & -16.210 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

The gripper is at $(11.55,\ 7.38,\ -16.21)$ cm. Its $z$ is $-8 - 8.5 \sin(90^\circ + 15^\circ) = -8 - 8.21 = -16.21$ cm, i.e. the forearm and the gripper both point down towards the plant. The nozzle is at $(15.26,\ 7.14,\ -8.00)$ cm.

## 7. Velocity kinematics: the Jacobian

The Jacobian $J$ maps joint velocities to the velocity of a point of the arm:

```math
\dot{p} = J_v(\theta)\,\dot{\theta}, \qquad \omega = J_\omega(\theta)\,\dot{\theta}, \qquad
J_v = \frac{\partial p}{\partial \theta} = \begin{bmatrix} \dfrac{\partial p}{\partial \theta_1} & \cdots & \dfrac{\partial p}{\partial \theta_4} \end{bmatrix}
```

For a revolute joint whose axis $z_{i-1}$ passes through $o_{i-1}$, the column is a cross product (the geometric Jacobian, notebooks 06–07):

```math
J_{v,i} = z_{i-1} \times (p - o_{i-1}), \qquad J_{\omega,i} = z_{i-1}
```

The joint axes of the D-H model are $z_0 = z_1 = (0, 0, 1)$ for J1 and J2, $z_2 = (-S_{12}, C_{12}, 0)$ for J3, and $z_4 = (S_{12}, -C_{12}, 0)$ for J4.

**Worked example.** The linear Jacobian of the gripper at the pose above, in cm/rad:

```math
J_{v,e2} = \begin{bmatrix}
-7.379 & -1.379 & -10.420 & 5.278 \\
11.549 & 1.157 & -12.418 & 6.290 \\
0 & 0 & 10.200 & -2.200
\end{bmatrix}
```

- Column 1 is $z_0 \times p = (-p_y,\ p_x,\ 0)$: turning J1 at 1 rad/s moves the gripper horizontally at $\sqrt{7.379^2 + 11.549^2} = 13.7$ cm/s.
- The last row has zeros in columns 1 and 2: the vertical joints cannot change the height.
- Turning J3 at 1 rad/s moves the gripper at 19.2 cm/s, the fastest of all joints, because it swings the forearm and the gripper.

**Singularities** are poses where $J_v$ loses rank and the arm cannot move in some direction. For the SCARA pair the determinant is $a_1 a_2 \sin\theta_2$, so the arm is singular when fully stretched or folded ($\theta_2 = 0$ or $\pm 180^\circ$). The straight-line moves of the ROS 2 software stop short of such poses: a line that would need a large joint jump is rejected (`_check_step` in `robot_interface.py`).

## 8. Inverse kinematics

Inverse kinematics finds the joint angles for a wanted position. For this arm it has a closed form, because the height depends on $\theta_3$ alone.

**Step 1: $\theta_3$ from the height of the wrist.**

```math
z = -a_3 S_3 - d_4 C_3
\;\;\Rightarrow\;\;
\rho \sin(\theta_3 + \varphi) = -z, \qquad \rho = \sqrt{a_3^2 + d_4^2},\ \ \varphi = \operatorname{atan2}(d_4, a_3)
```

**Step 2: the SCARA pair.** Projected onto the floor, the forearm lengthens the second link to

```math
L_2 = a_2 + a_3 C_3 - d_4 S_3
```

so $\theta_1, \theta_2$ follow from the two-link formula (notebook 19):

```math
\cos\theta_2 = \frac{x^2 + y^2 - a_1^2 - L_2^2}{2 a_1 L_2}, \qquad
\theta_2 = \operatorname{atan2}\!\left(\pm\sqrt{1 - \cos^2\theta_2},\ \cos\theta_2\right)
```

```math
\theta_1 = \operatorname{atan2}(y, x) - \operatorname{atan2}(L_2 \sin\theta_2,\ a_1 + L_2 \cos\theta_2)
```

The sign of the square root selects elbow-left or elbow-right.

**Step 3: $\theta_4$** sets the gripper pitch $\theta_3 - \theta_4$. For a wanted gripper position, first subtract the gripper offset $a_{4e2}$ along the wanted gripper direction, then solve steps 1–2 for the resulting wrist point.

**Worked example (the inverse of section 6).** The wrist point is $(12.964,\ 9.064,\ -8.000)$ cm.

1. $\rho = \sqrt{8^2 + 8^2} = 11.31$ and $\varphi = 45^\circ$, so $\sin(\theta_3 + 45^\circ) = 8/11.31 = 0.707$. This gives $\theta_3 = 0^\circ$ or $90^\circ$; we take $90^\circ$.
2. $L_2 = 12 + 8\cos 90^\circ - 8 \sin 90^\circ = 4$ cm.
3. $\cos\theta_2 = (12.964^2 + 9.064^2 - 12^2 - 4^2) / (2 \cdot 12 \cdot 4) = 90.23 / 96 = 0.940$, so $\theta_2 = 20.0^\circ$.
4. $\theta_1 = \operatorname{atan2}(9.064, 12.964) - \operatorname{atan2}(4 \sin 20^\circ,\ 12 + 4 \cos 20^\circ) = 34.96^\circ - 4.96^\circ = 30.0^\circ$.

The original angles come back. The notebook example ($x = 150$, $y = 100$ mm, $a_1 = a_2 = 100$ mm) gives $\cos\theta_2 = 0.625$, $\theta_2 = \operatorname{atan}(\sqrt{39}/5) = 51.32^\circ$ and $\theta_1 = 8.03^\circ$.

## 9. From the paper model to the ROS 2 robot

The robot in this repository is the next version of the paper's arm. Three things changed:

| | Paper model | ROS 2 robot (v2 CAD, `agrobot_ws/`) |
|---|---|---|
| Wrist J4 | parallel to J3, pitches the tool | perpendicular to J3, turns the gripper |
| End effectors | nozzle + gripper | one gripper; the nozzle is a possible extension |
| Geometry | D-H parameters | joint origins taken directly from the CAD (URDF) |

The software does not use D-H parameters. It computes the same forward kinematics by multiplying the fixed joint origins of the URDF with the joint rotations (Rodrigues' formula, $R = I + \sin\theta\,[\hat{z}]_\times + (1 - \cos\theta)[\hat{z}]_\times^2$), and the Jacobian with the cross-product rule of section 7.

Four joints cannot set all six numbers of a gripper pose (three for position, three for orientation). The robot therefore solves the position exactly and uses the remaining freedom to point the gripper as close as possible to the wanted approach direction $\hat{a}$, by damped least squares with a null-space task (`aibomech_agrobot_tasks/kinematics.py`):

```math
\Delta\theta = J_p^{+} e_p + \left(I - J_p^{+} J_p\right)(J_\omega N)^{+}\,(\hat{z}_{tcp} \times \hat{a}),
\qquad J^{+} = J^\top (J J^\top + \lambda I)^{-1}
```

- $e_p$: position error.
- $\hat{z}_{tcp} \times \hat{a}$: the rotation that would align the gripper axis with $\hat{a}$.
- $\lambda$: damping, which keeps the step bounded near singularities.
- $(I - J_p^+ J_p)$: projects the alignment step so it does not disturb the position.

Where this arm can reach, and from which direction, is the subject of the [workspace analysis](README.md#2-workspace).
