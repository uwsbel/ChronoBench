"""ANCF cable cantilever, turn 1 (CREATE): static tip deflection under a tip point load --
PyChrono 10.0 FEA, headless.

A slender CIRCULAR-section cantilever of length L modeled with ANCF gradient-deficient cable
elements (ChElementCableANCF via ChBuilderCableANCF), clamped at one end (first node's position AND
slope fixed), with a transverse tip load F. Nonlinear static solve. In the small-deflection regime
the ANCF cable statics must converge to Euler-Bernoulli: delta = F*L^3/(3*E*I), I = pi*d^4/64;
numbers chosen so delta ~= 0.42% of L: L=1, d=0.02, E=2e10, F=2 -> delta ~= 4.24e-3 m
(independent-oracle value).

Logs the deflected centerline (x, y for every node) to out.csv, so the judge MEASURES the tip
deflection from the shape (max |y|) instead of trusting a self-reported scalar.
"""
import csv
import json
import math

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

L = 1.0
N = 20
d = 0.02
E = 2.0e10
F = 2.0
rho = 7800.0
I = math.pi * d ** 4 / 64.0
delta_analytic = F * L ** 3 / (3.0 * E * I)

sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

sec = fea.ChBeamSectionCable()
sec.SetDiameter(d)
sec.SetYoungModulus(E)
sec.SetDensity(rho)
sec.SetRayleighDamping(0.0)

builder = fea.ChBuilderCableANCF()
builder.BuildBeam(mesh, sec, N, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(L, 0, 0))
nodes = builder.GetLastBeamNodes()
root = nodes[0]
tip = nodes[len(nodes) - 1]

root.SetFixed(True)             # clamps position AND slope of the ANCF node (cantilever root)
tip.SetForce(chrono.ChVector3d(0, -F, 0))

mesh.SetAutomaticGravity(False)
sys.Add(mesh)

sys.SetSolver(mkl.ChSolverPardisoMKL())
sys.DoStaticNonlinear(100)

tip_deflection = abs(tip.GetPos().y)
with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["x", "y"])
    for nd in nodes:
        p = nd.GetPos()
        w.writerow([f"{p.x:.6f}", f"{p.y:.6e}"])

print(json.dumps({"tip_deflection": tip_deflection, "analytic": delta_analytic}))
