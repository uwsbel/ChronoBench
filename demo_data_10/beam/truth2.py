"""Cantilever beam, turn 2 (MODIFY): static deflection under SELF-WEIGHT -- PyChrono 10.0 FEA, headless.

Same clamped cantilever as turn 1, but the tip point load is replaced by the beam's OWN WEIGHT: enable
automatic gravity on the FEA mesh and set g = 9.81 m/s^2 along -Y. This is a uniform distributed load
q = rho*A*g, for which the cantilever tip deflection is delta = q*L^4/(8*E*I) ~= 2.30e-3 m
(independent-oracle value). Logs the deflected centerline (x, y per node) to out.csv.
"""
import csv
import json

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

L = 1.0
N = 10
b = 0.05
E = 2.0e10
nu = 0.3
rho = 7800.0
g = 9.81
A = b * b
I = b ** 4 / 12.0
q = rho * A * g
delta_analytic = q * L ** 4 / (8.0 * E * I)

sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -g, 0))
mesh = fea.ChMesh()

sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsRectangularSection(b, b)
sec.SetYoungModulus(E)
sec.SetShearModulus(E / (2.0 * (1.0 + nu)))
sec.SetDensity(rho)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(mesh, sec, N, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(L, 0, 0),
                  chrono.ChVector3d(0, 1, 0))
nodes = builder.GetLastBeamNodes()
node0 = nodes[0]
tip = nodes[len(nodes) - 1]

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)
clamp = chrono.ChLinkMateGeneric()
clamp.Initialize(node0, truss, False, node0.Frame(), node0.Frame())
clamp.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(clamp)

mesh.SetAutomaticGravity(True)      # turn-2 change: self-weight instead of a tip point load
sys.Add(mesh)

sys.SetSolver(mkl.ChSolverPardisoMKL())
sys.DoStaticLinear()

tip_deflection = abs(tip.GetPos().y)
with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["x", "y"])
    for nd in nodes:
        p = nd.GetPos()
        w.writerow([f"{p.x:.6f}", f"{p.y:.6e}"])

print(json.dumps({"tip_deflection": tip_deflection, "analytic": delta_analytic}))
