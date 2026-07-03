"""A CORRECT-BUT-DIFFERENT turn-1 ANCF cantilever: same physics, different style. Builds the nodes
and ChElementCableANCF elements MANUALLY in a loop (no ChBuilderCableANCF), uses N=30 elements, the
MINRES iterative solver instead of Pardiso, and clamps the root node directly. Should pass L1/L2/L3
near ceiling."""
import csv
import json
import math

import pychrono as chrono
import pychrono.fea as fea

LENGTH = 1.0
NELEM = 30
DIAM = 0.02
E_MOD = 2.0e10
TIP_FORCE = 2.0
DENS = 7800.0

system = chrono.ChSystemSMC()
mesh = fea.ChMesh()

section = fea.ChBeamSectionCable()
section.SetDiameter(DIAM)
section.SetYoungModulus(E_MOD)
section.SetDensity(DENS)
section.SetRayleighDamping(0.0)

nodes = []
for i in range(NELEM + 1):
    x = LENGTH * i / NELEM
    node = fea.ChNodeFEAxyzD(chrono.ChVector3d(x, 0, 0), chrono.ChVector3d(1, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)

for i in range(NELEM):
    el = fea.ChElementCableANCF()
    el.SetNodes(nodes[i], nodes[i + 1])
    el.SetSection(section)
    mesh.AddElement(el)

nodes[0].SetFixed(True)                                   # cantilever clamp: position + slope
nodes[-1].SetForce(chrono.ChVector3d(0, -TIP_FORCE, 0))

mesh.SetAutomaticGravity(False)
system.Add(mesh)

solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(400)
solver.SetTolerance(1e-12)
system.SetSolver(solver)
system.DoStaticNonlinear(100)

with open("out.csv", "w", newline="") as fh:
    writer = csv.writer(fh)
    writer.writerow(["x", "y"])
    for node in nodes:
        p = node.GetPos()
        writer.writerow([f"{p.x:.6f}", f"{p.y:.6e}"])

inertia = math.pi * DIAM ** 4 / 64.0
print(json.dumps({"tip_deflection": abs(nodes[-1].GetPos().y),
                  "analytic": TIP_FORCE * LENGTH ** 3 / (3.0 * E_MOD * inertia)}))
