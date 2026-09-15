#!/usr/bin/env python3
"""
Generate a reduced-order symbolic arm dynamics reference.

This generator is deliberately offline and is not part of the default build. It
produces symbolic M(q), C(q,dq) and g(q) for a nominal planar 3R approximation.
Replace its parameters/kinematics with identified CHR values before using the
result in a controller. The runtime skeleton uses the exact MJCF frame chain only
for kinematics and does not pretend this reduced model is flight-ready.
"""

from pathlib import Path

import sympy as sp


def main() -> None:
    q = sp.Matrix(sp.symbols('q1:4', real=True))
    dq = sp.Matrix(sp.symbols('dq1:4', real=True))
    lengths = sp.symbols('l1:4', positive=True)
    masses = sp.symbols('m1:4', positive=True)
    inertias = sp.symbols('I1:4', positive=True)
    gravity = sp.symbols('g', positive=True)

    points = []
    angle = 0
    origin = sp.Matrix([0, 0])
    for index in range(3):
        angle += q[index]
        direction = sp.Matrix([sp.cos(angle), sp.sin(angle)])
        points.append(origin + lengths[index] * direction / 2)
        origin += lengths[index] * direction

    kinetic = 0
    potential = 0
    cumulative_rate = 0
    for index, point in enumerate(points):
        velocity = point.jacobian(q) * dq
        cumulative_rate += dq[index]
        kinetic += sp.Rational(1, 2) * masses[index] * velocity.dot(velocity)
        kinetic += sp.Rational(1, 2) * inertias[index] * cumulative_rate ** 2
        potential += masses[index] * gravity * point[1]

    mass_matrix = sp.simplify(sp.hessian(kinetic, dq))
    gravity_vector = sp.simplify(sp.Matrix([sp.diff(potential, coordinate) for coordinate in q]))
    coriolis = sp.zeros(3, 1)
    for i in range(3):
        for j in range(3):
            for k in range(3):
                christoffel = sp.Rational(1, 2) * (
                    sp.diff(mass_matrix[i, j], q[k]) +
                    sp.diff(mass_matrix[i, k], q[j]) -
                    sp.diff(mass_matrix[j, k], q[i]))
                coriolis[i] += christoffel * dq[j] * dq[k]

    output = Path(__file__).with_name('generated_chr_dynamics.txt')
    output.write_text(
        '# Nominal planar 3R model; validate before control use.\n\n'
        f'M(q) =\n{sp.sstr(mass_matrix)}\n\n'
        f'C(q,dq) =\n{sp.sstr(sp.simplify(coriolis))}\n\n'
        f'g(q) =\n{sp.sstr(gravity_vector)}\n',
        encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
