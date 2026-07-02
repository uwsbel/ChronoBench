"""Cantilever beam, turn 3 (EXTEND): superposition of a tip load and a mid-span load -- 10.0 FEA, headless.

Same clamped cantilever as turn 1, but with TWO transverse point loads: the original tip load F and an
equal load P=F applied at the mid-span node (x=L/2). By superposition the tip deflection is
delta = F*L^3/(3*E*I) + 5*P*L^3/(48*E*I) ~= 6.3e-3 m (independent-oracle value). Tests locating and
loading an interior node plus linear superposition. Logs the deflected centerline (x, y per node).
"""
import csv
import json

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

L = 1.0
N = 10                       # even -> node N/2 sits exactly at the mid-span x = L/2
b = 0.05
E = 2.0e10
nu = 0.3
F = 150.0                    # tip load; equal load P = F at mid-span
rho = 7800.0
I = b ** 4 / 12.0
delta_analytic = F * L ** 3 / (3.0 * E * I) + 5.0 * F * L ** 3 / (48.0 * E * I)

sys = chrono.ChSystemSMC()
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
mid = nodes[len(nodes) // 2]     # x = L/2
tip = nodes[len(nodes) - 1]

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)
clamp = chrono.ChLinkMateGeneric()
clamp.Initialize(node0, truss, False, node0.Frame(), node0.Frame())
clamp.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(clamp)

tip.SetForce(chrono.ChVector3d(0, -F, 0))
mid.SetForce(chrono.ChVector3d(0, -F, 0))     # turn-3 addition: equal load at mid-span
mesh.SetAutomaticGravity(False)
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
