"""Cantilever beam, static tip deflection (PyChrono 10.0 FEA, headless) -- contracted reference.

A slender square-section Euler-Bernoulli beam of length L, clamped at one end, with a transverse tip
load F. Linear static solve (Pardiso). Reports the tip deflection, which for a cantilever is the
textbook delta = F*L^3/(3*E*I), with I = b^4/12 for a square b x b section. Numbers chosen so the
deflection is ~0.5% of L (linear/Euler regime): L=1, b=0.05, E=2e10, F=150 -> delta ~= 4.8e-3 m.
"""
import json

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

L = 1.0
N = 10
b = 0.05
E = 2.0e10
nu = 0.3
F = 150.0
rho = 7800.0
I = b * b ** 3 / 12.0
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
node0 = nodes[0]
tip = nodes[len(nodes) - 1]

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)
clamp = chrono.ChLinkMateGeneric()
clamp.Initialize(node0, truss, False, node0.Frame(), node0.Frame())
clamp.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(clamp)

tip.SetForce(chrono.ChVector3d(0, -F, 0))
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

sys.SetSolver(mkl.ChSolverPardisoMKL())
sys.DoStaticLinear()

tip_deflection = abs(tip.GetPos().y)
print(json.dumps({"tip_deflection": tip_deflection, "analytic": delta_analytic}))
