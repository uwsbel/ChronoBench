"""A CORRECT turn-1 cantilever tip-deflection in a deliberately different style (N=8, different naming and
shape-logging). Same physics as the reference -> should PASS. Shows the judge does not penalize stylistic
divergence (and that any reasonable element count gives the same Euler-Bernoulli tip deflection)."""
import csv
import json

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

length, nel, side, ymod, poisson, load, dens = 1.0, 8, 0.05, 2.0e10, 0.3, 150.0, 7800.0

system = chrono.ChSystemSMC()
fem = fea.ChMesh()
section = fea.ChBeamSectionEulerAdvanced()
section.SetAsRectangularSection(side, side)
section.SetYoungModulus(ymod)
section.SetShearModulus(ymod / (2.0 * (1.0 + poisson)))
section.SetDensity(dens)

bld = fea.ChBuilderBeamEuler()
bld.BuildBeam(fem, section, nel, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(length, 0, 0),
              chrono.ChVector3d(0, 1, 0))
pts = bld.GetLastBeamNodes()

anchor = chrono.ChBody()
anchor.SetFixed(True)
system.Add(anchor)
weld = chrono.ChLinkMateGeneric()
weld.Initialize(pts[0], anchor, False, pts[0].Frame(), pts[0].Frame())
weld.SetConstrainedCoords(True, True, True, True, True, True)
system.Add(weld)

pts[-1].SetForce(chrono.ChVector3d(0, -load, 0))
fem.SetAutomaticGravity(False)
system.Add(fem)
system.SetSolver(mkl.ChSolverPardisoMKL())
system.DoStaticLinear()

with open("out.csv", "w", newline="") as fh:
    wr = csv.writer(fh)
    wr.writerow(["x", "y"])
    for nd in pts:
        pos = nd.GetPos()
        wr.writerow(["%.6f" % pos.x, "%.6e" % pos.y])

Imom = side ** 4 / 12.0
print(json.dumps({"tip_deflection": abs(pts[-1].GetPos().y),
                  "analytic": load * length ** 3 / (3.0 * ymod * Imom)}))
