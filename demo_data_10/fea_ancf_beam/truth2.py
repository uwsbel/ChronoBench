"""ANCF cable cantilever, turn 2 (MODIFY): static deflection under self-weight --
PyChrono 10.0 FEA, headless.

Same circular-section ANCF cable cantilever as turn 1, but the tip load is removed and the beam
now sags under its OWN WEIGHT: gravity 9.81 m/s^2 along -Y with the mesh's automatic gravity load
enabled (a uniform line load q = rho*A*g in Euler-Bernoulli terms). Nonlinear static solve. The
small-deflection reference is delta = q*L^4/(8*E*I) ~= 1.91e-2 m (independent-oracle value).

Logs the deflected centerline (x, y for every node) to out.csv, so the judge MEASURES the tip
deflection from the shape (max |y|).
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
rho = 7800.0
g = 9.81
A = math.pi * d ** 2 / 4.0
I = math.pi * d ** 4 / 64.0
q = rho * A * g
delta_analytic = q * L ** 4 / (8.0 * E * I)

sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -g, 0))
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

mesh.SetAutomaticGravity(True)  # self-weight is the ONLY load this turn
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
