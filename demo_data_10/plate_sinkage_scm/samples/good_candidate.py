"""A CORRECT turn-1 plate-sinkage-on-SCM in a deliberately different style (different naming/structure;
sinkage tracked via the plate's absolute Z rather than an accumulated drop). Same physics params as the
reference -> the settled sinkage lands in the Bekker band -> should PASS. Shows the judge does not penalize
stylistic divergence, only that the plate really sinks into SCM by the right amount."""
import csv
import json

import pychrono as chrono
import pychrono.vehicle as veh

# same physics parameters as the reference (different code style)
soil = dict(Kphi=2.0e6, Kc=0.0, n=1.0, coh=0.0, fric=30.0, jan=0.01, eK=2.0e7, damp=1.0e4)
side_x = side_y = 0.2
thick = 0.05
load = 500.0
grav = 9.81
step, stop = 2.0e-3, 1.5

world = chrono.ChSystemSMC()
world.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
world.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -grav))

soil_patch = veh.SCMTerrain(world, False)
soil_patch.SetSoilParameters(soil["Kphi"], soil["Kc"], soil["n"], soil["coh"], soil["fric"],
                             soil["jan"], soil["eK"], soil["damp"])
soil_patch.Initialize(1.0, 1.0, 0.02)

surf = chrono.ChContactMaterialSMC()
surf.SetFriction(0.8)
surf.SetYoungModulus(1.0e7)

foot = chrono.ChBodyEasyBox(side_x, side_y, thick, (load / grav) / (side_x * side_y * thick), True, True, surf)
foot.SetPos(chrono.ChVector3d(0, 0, thick / 2.0))
world.Add(foot)
top0 = foot.GetPos().z

times, drops = [], []
while world.GetChTime() < stop:
    soil_patch.Synchronize(world.GetChTime())
    world.DoStepDynamics(step)
    soil_patch.Advance(step)
    times.append(world.GetChTime())
    drops.append(top0 - foot.GetPos().z)

with open("out.csv", "w", newline="") as fh:
    wr = csv.writer(fh)
    wr.writerow(["t", "sinkage"])
    for tt, dd in zip(times, drops):
        wr.writerow(["%.5f" % tt, "%.6e" % dd])

print(json.dumps({"sinkage": drops[-1], "load_N": load}))
