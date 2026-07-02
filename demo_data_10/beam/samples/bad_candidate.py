"""A RUNS-BUT-WRONG turn-1 cantilever: structurally fine and it executes cleanly (passes L1 and the
minimal L2 capability checks), but it uses Young's modulus 2e11 Pa instead of the specified 2e10 (10x too
stiff, a units slip), so the tip deflection is ~10x too small. The shape-derived L3 invariant (max |y|)
catches it and the wrong-physics cap applies."""
import csv
import json

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

L = 1.0
N = 10
b = 0.05
E = 2.0e11          # WRONG: should be 2.0e10 (10x too stiff)
nu = 0.3
F = 150.0
rho = 7800.0
I = b ** 4 / 12.0
delta_analytic = F * L ** 3 / (3.0 * E * I)

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
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)
clamp = chrono.ChLinkMateGeneric()
clamp.Initialize(nodes[0], truss, False, nodes[0].Frame(), nodes[0].Frame())
clamp.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(clamp)
nodes[-1].SetForce(chrono.ChVector3d(0, -F, 0))
mesh.SetAutomaticGravity(False)
sys.Add(mesh)
sys.SetSolver(mkl.ChSolverPardisoMKL())
sys.DoStaticLinear()

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["x", "y"])
    for nd in nodes:
        p = nd.GetPos()
        w.writerow([f"{p.x:.6f}", f"{p.y:.6e}"])
print(json.dumps({"tip_deflection": abs(nodes[-1].GetPos().y), "analytic": delta_analytic}))
