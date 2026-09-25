#!/usr/bin/env python3
"""Symbolic kinematics of the thesis arm model (Mathematica notebooks 01-07, 19).

Rebuilds, with SymPy, the equations of the paper (Altawil & Can, IJAMEC 2023)
and of the notebooks in analysis/mathematica/:
  * Denavit-Hartenberg table and homogeneous transforms T_0i   (paper, Table 1)
  * forward kinematics of both end effectors, checked against
    the expressions printed in the paper                      (paper, groups 3-4)
  * rotation matrices R_i0 of the link frames          (notebook 01)
  * joint axis directions z_i = R_i0 · [0 0 1]ᵀ        (notebook 02)
  * centre-of-mass positions p_ci(q)                   (notebooks 03-04)
  * linear Jacobians J_vi = ∂p_ci/∂q                   (notebooks 05-07)
  * two-link inverse kinematics (the SCARA plane)      (notebook 19)
  * closed-form inverse kinematics of the wrist point  (θ3 from the height, then θ1, θ2)

Link lengths a1..a4 and offset d3 are those of the thesis (cm).

    python3 01_kinematics_thesis_model.py
"""
import sympy as sp

q1, q2, q3, q4 = q = sp.symbols('q1 q2 q3 q4', real=True)
a1, a2, a3, a4, d3 = sp.symbols('a1 a2 a3 a4 d3', positive=True)
c, s = sp.cos, sp.sin

# Link frame orientations relative to the base frame (notebook 01).
# Joints 1 and 2 turn about the vertical axis, joint 3 pitches, joint 4 is the wrist.
R10 = sp.Matrix([[c(q1), -s(q1), 0], [s(q1), c(q1), 0], [0, 0, 1]])
R20 = sp.Matrix([[c(q1 + q2), 0, -s(q1 + q2)], [s(q1 + q2), 0, c(q1 + q2)], [0, -1, 0]])
R30 = sp.Matrix([[c(q3) * c(q1 + q2), s(q1 + q2), -s(q3) * c(q1 + q2)],
                 [c(q3) * s(q1 + q2), -c(q1 + q2), -s(q3) * s(q1 + q2)],
                 [-s(q3), 0, -c(q3)]])
R40 = sp.Matrix([[c(q3 - q4) * c(q1 + q2), s(q3 - q4) * c(q1 + q2), s(q1 + q2)],
                 [c(q3 - q4) * s(q1 + q2), s(q3 - q4) * s(q1 + q2), c(q1 + q2)],
                 [-s(q3 - q4), c(q3 - q4), 0]])
z_hat = sp.Matrix([0, 0, 1])

# Centre of mass of every link in the base frame (notebook 04): each link is a
# uniform rod, so its COM is at half its length along the link.
p_c1 = sp.Matrix([a1 / 2 * c(q1), a1 / 2 * s(q1), 0])
p_c2 = sp.Matrix([a1 * c(q1) + a2 / 2 * c(q1 + q2), a1 * s(q1) + a2 / 2 * s(q1 + q2), 0])
elbow = sp.Matrix([a1 * c(q1) + a2 * c(q1 + q2), a1 * s(q1) + a2 * s(q1 + q2), 0])
p_c3 = elbow + R30 * sp.Matrix([a3 / 2, 0, d3 / 2])
p_c4 = elbow + R30 * sp.Matrix([a3, 0, d3]) + R40 * sp.Matrix([a4 / 2, 0, 0])


# ---------------------------------------------------------------- D-H model --
# Classical Denavit-Hartenberg convention, Table 1 of the paper:
#   A_i = Rot_z(θ_i) · Trans_z(d_i) · Trans_x(a_i) · Rot_x(α_i)
# Rows 4e1 and 4e2 are the two end effectors on the wrist: the fertiliser
# nozzle (e1, along the wrist axis) and the gripper (e2, across it).
d4, d4e1, a4e2 = sp.symbols('d4 d_4e1 a_4e2', positive=True)
DH_TABLE = [  # name, a_i, α_i, d_i, θ_i
    ('1', a1, 0, 0, q1),
    ('2', a2, -sp.pi / 2, 0, q2),
    ('3', a3, -sp.pi / 2, 0, q3),
    ('4', 0, -sp.pi / 2, d4, 0),
    ('4e1', 0, 0, d4e1, q4),
    ('4e2', a4e2, 0, 0, q4),
]


def dh_matrix(a, alpha, d, theta):
    """Homogeneous transform of one D-H row (equation 2 of the paper)."""
    ct, st, ca, sa = c(theta), s(theta), c(alpha), s(alpha)
    return sp.Matrix([[ct, -st * ca, st * sa, a * ct],
                      [st, ct * ca, -ct * sa, a * st],
                      [0, sa, ca, d],
                      [0, 0, 0, 1]])


def dh_transforms():
    """T_01, T_02, T_03, T_04 and the end-effector transforms T_0e1, T_0e2."""
    rows = {name: dh_matrix(a, al, d, th) for name, a, al, d, th in DH_TABLE}
    t = {'T01': rows['1']}
    t['T02'] = sp.simplify(t['T01'] * rows['2'])
    t['T03'] = sp.simplify(t['T02'] * rows['3'])
    t['T04'] = sp.simplify(t['T03'] * rows['4'])
    t['T0e1'] = sp.simplify(t['T04'] * rows['4e1'])
    t['T0e2'] = sp.simplify(t['T04'] * rows['4e2'])
    return t


def paper_positions():
    """End-effector positions as printed in the paper (groups 1-4), with
    beta = C12 C3, gamma = C12 S3, delta = S12 C3, eps = S12 S3, forall = C12 C34."""
    c12, s12 = c(q1 + q2), s(q1 + q2)
    beta, gamma, delta, eps = c12 * c(q3), c12 * s(q3), s12 * c(q3), s12 * s(q3)
    forall, s34 = c12 * c(q3 - q4), s(q3 - q4)
    base_x = a1 * c(q1) + a2 * c12 + a3 * beta - d4 * gamma
    base_y = a1 * s(q1) + a2 * s12 + a3 * delta - d4 * eps
    base_z = -a3 * s(q3) - d4 * c(q3)
    pe1 = sp.Matrix([base_x + d4e1 * s12, base_y - d4e1 * c12, base_z])
    # For e2 the D-H product gives a_4e2 · ∀ (= C12 C34); the text export of the
    # paper shows β in its place.
    pe2 = sp.Matrix([base_x + a4e2 * forall, base_y + a4e2 * s12 * c(q3 - q4), base_z - a4e2 * s34])
    return pe1, pe2


def jacobian(p):
    """Linear Jacobian J_v = ∂p/∂q (3 x 4)."""
    return sp.simplify(p.jacobian(sp.Matrix(q)))


def two_link_ik(x, y, l1, l2):
    """Planar two-link inverse kinematics (notebook 19), elbow-up solution.

    cos(θ2) = (x² + y² − l1² − l2²) / (2 l1 l2),  θ2 = atan2(√(1−cos²θ2), cos θ2)
    θ1 = atan2(y, x) − atan2(l2 sin θ2, l1 + l2 cos θ2)
    """
    c2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    theta2 = sp.atan2(sp.sqrt(1 - c2 ** 2), c2)
    theta1 = sp.atan2(y, x) - sp.atan2(l2 * sp.sin(theta2), l1 + l2 * sp.cos(theta2))
    return sp.simplify(theta1), sp.simplify(theta2)


def wrist_ik(x, y, z, l1, l2, l3, off):
    """Closed-form IK of the wrist point (origin of frame 4) of the D-H model.

    The height depends on θ3 only:   z = −a3 sin θ3 − d4 cos θ3
    and the horizontal position is a two-link arm whose second link has the
    effective length  L2 = a2 + a3 cos θ3 − d4 sin θ3.
    Returns (θ1, θ2, θ3) in radians, choosing the θ3 in [0, π) and elbow-up θ2 ≥ 0.
    """
    rho, phi = sp.sqrt(l3 ** 2 + off ** 2), sp.atan2(off, l3)
    theta3 = sp.pi - sp.asin(-z / rho) - phi            # a3 S3 + d4 C3 = ρ sin(θ3 + φ) = −z
    if not 0 <= float(theta3) < float(sp.pi):
        theta3 = sp.asin(-z / rho) - phi
    length2 = l2 + l3 * sp.cos(theta3) - off * sp.sin(theta3)
    theta1, theta2 = two_link_ik(x, y, l1, length2)
    return theta1, theta2, theta3


def main():
    print('Denavit-Hartenberg transforms (paper, Table 1)')
    t = dh_transforms()
    for name in ('T02', 'T03', 'T04', 'T0e1', 'T0e2'):
        print(f'  {name}: position = {[sp.simplify(e) for e in t[name][:3, 3]]}')
    pe1, pe2 = paper_positions()
    ok1 = sp.simplify(t['T0e1'][:3, 3] - pe1) == sp.zeros(3, 1)
    ok2 = sp.simplify(t['T0e2'][:3, 3] - pe2) == sp.zeros(3, 1)
    print(f'  matches the paper: end effector 1 {ok1}, end effector 2 {ok2}')
    # Notebook 01 writes R40 with +cos(q1+q2) in row 2, column 3; the D-H product
    # gives -cos(q1+q2). Only the D-H version is a rotation matrix (R^T R = I).
    diff = sp.simplify(t['T0e1'][:3, :3] - R40)
    print(f'  R40 of notebook 01 differs from the D-H rotation in: '
          f'{[(r + 1, k + 1) for r in range(3) for k in range(3) if diff[r, k] != 0]}')
    print(f'  notebook R40 orthogonal: {sp.simplify(R40.T * R40) == sp.eye(3)}, '
          f'D-H rotation orthogonal: {sp.simplify(t["T0e1"][:3, :3].T * t["T0e1"][:3, :3]) == sp.eye(3)}')

    sample = {q1: sp.rad(30), q2: sp.rad(20), q3: sp.rad(90), q4: sp.rad(-15),
              a1: 12, a2: 12, a3: 8, d4: 8, a4e2: 8.5, d4e1: 3}
    print('\nWorked example: θ = (30°, 20°, 90°, −15°), a1 = a2 = 12, a3 = d4 = 8, '
          'a_4e2 = 8.5, d_4e1 = 3 cm')
    for name in ('T02', 'T03', 'T04', 'T0e2'):
        num = t[name].subs(sample).evalf(4)
        print(f'  {name} =')
        for r in range(4):
            print('     [' + '  '.join(f'{float(v):8.3f}' for v in num[r, :]) + ']')
    print(f'  gripper (e2) position = {[round(float(v), 3) for v in pe2.subs(sample)]} cm')
    print(f'  nozzle  (e1) position = {[round(float(v), 3) for v in pe1.subs(sample)]} cm')
    je2 = pe2.jacobian(sp.Matrix(q)).subs(sample)
    print('  Jacobian of the gripper position J_v,e2 = ∂p_e2/∂θ (cm/rad) =')
    for r in range(3):
        print('     [' + '  '.join(f'{float(v):8.3f}' for v in je2[r, :]) + ']')

    wrist = t['T04'][:3, 3].subs(sample)
    ik = wrist_ik(*[float(v) for v in wrist], 12, 12, 8, 8)
    print(f'  inverse kinematics of the wrist point {[round(float(v), 3) for v in wrist]}: '
          f'θ1, θ2, θ3 = {[round(float(sp.deg(v)), 2) for v in ik]}°')

    print()
    print('Joint axis directions z_i = R_i0 · ẑ (notebook 02)')
    for name, rot in (('z1', R10), ('z2', R20), ('z3', R30), ('z4', R40)):
        print(f'  {name} = {list(rot * z_hat)}')

    print('\nCentre-of-mass positions (notebook 04)')
    for i, p in enumerate((p_c1, p_c2, p_c3, p_c4), 1):
        print(f'  p_c{i} = {[sp.simplify(e) for e in p]}')

    print('\nLinear Jacobians J_vi = ∂p_ci/∂q (notebook 07), LaTeX of J_v2:')
    print('  ' + sp.latex(jacobian(p_c2)))
    for i, p in enumerate((p_c1, p_c2, p_c3, p_c4), 1):
        jv = jacobian(p)
        print(f'  J_v{i}: {jv.shape}, non-zero columns: {[k + 1 for k in range(4) if any(jv[:, k])]}')

    print('\nTwo-link IK example of notebook 19: x = 150, y = 100, l1 = l2 = 100')
    t1, t2 = two_link_ik(150, 100, 100, 100)
    print(f'  θ2 = {t2} = {float(sp.deg(t2)):.2f}°   (notebook: ArcTan[√39/5])')
    print(f'  θ1 = {float(sp.deg(t1)):.2f}°')
    # Forward check: the elbow and tip must land on the target.
    x = 100 * sp.cos(t1) + 100 * sp.cos(t1 + t2)
    y = 100 * sp.sin(t1) + 100 * sp.sin(t1 + t2)
    print(f'  forward check: x = {float(x):.3f}, y = {float(y):.3f}')


if __name__ == '__main__':
    main()
