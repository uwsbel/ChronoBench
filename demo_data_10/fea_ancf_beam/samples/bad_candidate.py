"""A RUNS-BUT-WRONG turn-1 ANCF cantilever: structurally fine and it executes cleanly (passes L1 and
the minimal L2 capability checks), but it uses E = 2.0e11 Pa (steel's handbook value; a units/spec
slip, the prompt says 2.0e10), so the beam is 10x too stiff and the tip deflection is ~4.2e-4 m
instead of ~4.2e-3 m. The CSV-derived L3 invariant catches it and the wrong-physics cap applies."""
import csv
import json
import math

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

L = 1.0
N = 20
d = 0.02
E = 2.0e11                     # WRONG: should be 2.0e10
F = 2.0
rho = 7800.0
I = math.pi * d ** 4 / 64.0

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
nodes[0].SetFixed(True)
nodes[len(nodes) - 1].SetForce(chrono.ChVector3d(0, -F, 0))

mesh.SetAutomaticGravity(False)
sys.Add(mesh)

sys.SetSolver(mkl.ChSolverPardisoMKL())
sys.DoStaticNonlinear(100)

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["x", "y"])
    for nd in nodes:
        p = nd.GetPos()
        w.writerow([f"{p.x:.6f}", f"{p.y:.6e}"])

print(json.dumps({"tip_deflection": abs(nodes[len(nodes) - 1].GetPos().y),
                  "analytic": F * L ** 3 / (3.0 * E * I)}))
