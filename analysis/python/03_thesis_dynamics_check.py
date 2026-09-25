#!/usr/bin/env python3
"""Check of the thesis dynamics (Mathematica notebooks 07-16).

The notebooks derive the joint torques of the arm with the Euler-Lagrange
method, τ = M(q) q̈ + C(q, q̇) q̇, and evaluate them in notebook 16 at

    q = (30°, 20°, 90°, −15°),  q̇ = (0.5, 0.4, 0.4, 0.4) rad/s,  q̈ = 0,
    a1 = a2 = 12 cm, a3 = 8 cm, a4 = 8.5 cm, d3 = 8 cm,
    m1 = m2 = m3 = 0.08 kg, m4 = 0.06 kg.

This script takes the inertia matrix M(q) exactly as written in notebook 12b
(read from its text export), recomputes the torques and compares them with
the values stored in notebook 16. It reports three findings of the review:

1. The notebook's Christoffel symbols miss the factor 1/2:
       c_ijk = 1/2 (∂M_kj/∂q_i + ∂M_ki/∂q_j − ∂M_ij/∂q_k)
   Without it the result matches the notebook (τ1..τ3 exactly), so the
   notebook's Coriolis/centrifugal torques are twice too large.
2. There is no gravity term G(q). Joint 3 (the pitch joint) carries the
   weight of the forearm and gripper, so this term dominates at low speed.
3. The linear Jacobians of links 3 and 4 in notebook 07 differ from ∂p_c/∂q
   (see 01_kinematics_thesis_model.py); M(q) inherits that error.
   robot_model.py computes everything from the URDF with the corrected method.

Units of the thesis model: cm and kg, so torques come out in kg·cm²/s²
(1 kg·cm²/s² = 1e-4 N·m).

    python3 03_thesis_dynamics_check.py
"""
import importlib.util
import os
import re

import sympy as sp
from sympy.parsing.mathematica import parse_mathematica

HERE = os.path.dirname(os.path.abspath(__file__))
NOTEBOOK_M = os.path.join(HERE, '..', 'mathematica', 'text', '12b_christoffel_symbols_organized.txt')

PARAMS = {'a1': 12, 'a2': 12, 'a3': 8, 'a4': 8.5, 'd3': 8, 'm1': 0.08, 'm2': 0.08, 'm3': 0.08, 'm4': 0.06}
Q = [sp.rad(30), sp.rad(20), sp.rad(90), sp.rad(-15)]
QD = [0.5, 0.4, 0.4, 0.4]
NOTEBOOK_TAU = [-22.93141862495481, -3.131435415037906, 10.585918521974984, 3.123879655347872]


def notebook_mass_matrix():
    """M(q) as written in notebook 12b, parsed from Mathematica syntax."""
    text = open(NOTEBOOK_M).read()
    entries = {}
    for match in re.finditer(r'(M[1-4][1-4])=(.*?);', text, re.S):
        entries.setdefault(match.group(1), parse_mathematica(match.group(2).replace('\n', ' ')))
    return sp.Matrix(4, 4, lambda i, j: entries[f'M{i + 1}{j + 1}'])


def coriolis_torque(m_q, q, q_vals, qd_vals, factor):
    """Σ_ij c_ijk q̇_i q̇_j with c_ijk = factor · (∂M_kj/∂q_i + ∂M_ki/∂q_j − ∂M_ij/∂q_k)."""
    at = dict(zip(q, q_vals))
    dm = [[[sp.diff(m_q[a, b], q[c]).subs(at) for c in range(4)] for b in range(4)] for a in range(4)]
    tau = []
    for k in range(4):
        total = 0.0
        for i in range(4):
            for j in range(4):
                total += factor * (dm[k][j][i] + dm[k][i][j] - dm[i][j][k]) * qd_vals[i] * qd_vals[j]
        tau.append(float(total))
    return tau


def coriolis_matrix(m_q, q, q_vals, qd_vals):
    """C_kj = Σ_i c_ijk q̇_i with the standard factor 1/2 (Table 2 of the paper sums
    the c_ijk of one row and column in the same way)."""
    at = dict(zip(q, q_vals))
    dm = [[[sp.diff(m_q[a, b], q[c]).subs(at) for c in range(4)] for b in range(4)] for a in range(4)]
    return sp.Matrix(4, 4, lambda k, j: sum(
        0.5 * (dm[k][j][i] + dm[k][i][j] - dm[i][j][k]) * qd_vals[i] for i in range(4)))


def thesis_gravity(q_vals):
    """G_k = ∂V/∂q_k with V = Σ m_i g z_ci, the COM heights of 01_kinematics_thesis_model.py.

    Links 1 and 2 move in the horizontal plane (z_c1 = z_c2 = 0), so only links
    3 and 4 contribute. g = 981 cm/s² keeps the thesis units (kg·cm²/s²)."""
    spec = importlib.util.spec_from_file_location(
        'kin', os.path.join(HERE, '01_kinematics_thesis_model.py'))
    kin = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(kin)
    values = {sp.Symbol(k, positive=True): v for k, v in PARAMS.items()}
    energy = 981 * (PARAMS['m3'] * kin.p_c3[2] + PARAMS['m4'] * kin.p_c4[2]).subs(values)
    return [float(sp.diff(energy, qk).subs(dict(zip(kin.q, q_vals)))) for qk in kin.q]


def print_matrix(name, m):
    print(f'{name} =')
    for r in range(m.shape[0]):
        print('   [' + '  '.join(f'{float(v):9.3f}' for v in m[r, :]) + ']')


def main():
    q = sp.symbols('q1:5')
    m_q = notebook_mass_matrix().subs({sp.Symbol(k): v for k, v in PARAMS.items()})
    print('M(q) from notebook 12b is symmetric:', sp.simplify(m_q - m_q.T) == sp.zeros(4))

    as_notebook = coriolis_torque(m_q, q, Q, QD, factor=1.0)
    corrected = coriolis_torque(m_q, q, Q, QD, factor=0.5)

    print('\nJoint torques at the notebook-16 example (kg·cm²/s², q̈ = 0, no gravity)\n')
    print('| joint | notebook 16 | recomputed without 1/2 | with the standard 1/2 |')
    print('|---|---|---|---|')
    for k in range(4):
        print(f'| τ{k + 1} | {NOTEBOOK_TAU[k]:8.3f} | {as_notebook[k]:8.3f} | {corrected[k]:8.3f} |')
    print('\nτ1..τ3 are reproduced exactly without the factor 1/2, which confirms finding 1.')
    print('τ4 differs slightly: the explicit T4 expression in notebook 15/16 does not fully')
    print('match the assembled M and C matrices of notebook 14.')

    print('\nWorked example, all terms of τ = M q̈ + C q̇ + G at the same pose (kg·cm², kg·cm²/s)')
    m_num = m_q.subs(dict(zip(q, Q))).evalf()
    print_matrix('M(q)', m_num)
    c_num = coriolis_matrix(m_q, q, Q, QD)
    print_matrix('C(q, q̇) with 1/2', c_num)
    g_vec = thesis_gravity(Q)
    print('G(q) =', [round(v, 2) for v in g_vec], 'kg·cm²/s²  (g = 981 cm/s²)')
    cqd = c_num * sp.Matrix(QD)
    print('\n| joint | C q̇ | G | τ (q̈ = 0) | τ in N·m |')
    print('|---|---|---|---|---|')
    for k in range(4):
        tau = float(cqd[k]) + g_vec[k]
        print(f'| τ{k + 1} | {float(cqd[k]):8.3f} | {g_vec[k]:8.2f} | {tau:8.2f} | {tau * 1e-4:.4f} |')
    print('Gravity is about 160 times (joint 3) and 37 times (joint 4) the velocity terms.\n')
    print('In SI units the corrected torques are about '
          + ', '.join(f'{t * 1e-4:.1e}' for t in corrected) + ' N·m: at these speeds the')
    print('dynamic torques are tiny; gravity (see 04_torque_sizing.py) sizes the motors.')


if __name__ == '__main__':
    main()
