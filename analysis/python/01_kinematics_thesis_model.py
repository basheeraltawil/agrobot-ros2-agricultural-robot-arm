#!/usr/bin/env python3
"""Symbolic kinematics of the thesis arm model (Mathematica notebooks 01-07, 19).

Rebuilds, with SymPy, the equations derived in analysis/mathematica/:
  * rotation matrices R_i0 of the link frames          (notebook 01)
  * joint axis directions z_i = R_i0 · [0 0 1]ᵀ        (notebook 02)
  * centre-of-mass positions p_ci(q)                   (notebooks 03-04)
  * linear Jacobians J_vi = ∂p_ci/∂q                   (notebooks 05-07)
  * two-link inverse kinematics (the SCARA plane)      (notebook 19)

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


def main():
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
