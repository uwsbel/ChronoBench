"""A RUNS-BUT-WRONG turn-1 contact reporter: structurally fine and it executes cleanly (passes L1 and the
minimal L2 capability checks, the callback subclass works), but it drops only N=2 spheres instead of the
specified 4. The CSV-derived invariants catch it: the contact count is too low and the summed normal force
(~2*m*g) is well below the expected 4*m*g. The wrong-physics cap applies."""
import csv
import json
import math

import pychrono as chrono

N = 2                        # WRONG: should be 4 spheres
radius = 0.1
density = 1.0 / ((4.0 / 3.0) * math.pi * radius ** 3)
g = 9.81
t_end, dt = 1.5, 1.0e-3

sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -g, 0.0))
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.4)
mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(4.0, 0.2, 4.0, 1000.0, True, True, mat)
ground.SetPos(chrono.ChVector3d(0.0, -0.1, 0.0))
ground.SetFixed(True)
sys.AddBody(ground)
for i in range(N):
    s = chrono.ChBodyEasySphere(radius, density, True, True, mat)
    s.SetPos(chrono.ChVector3d(-0.6 + 0.4 * i, radius + 0.05, 0.0))
    sys.AddBody(s)
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)


class ContactReporter(chrono.ReportContactCallback):
    def __init__(self):
        super().__init__()
        self.forces = []

    def OnReportContact(self, pA, pB, plane_coord, distance, eff_radius, react_forces,
                        react_torques, modA, modB, contact_id):
        self.forces.append(abs(react_forces.x))
        return True


reporter = ContactReporter()
sys.GetContactContainer().ReportAllContacts(reporter)
with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["contact_id", "normal_force"])
    for i, fn in enumerate(reporter.forces):
        w.writerow([i, "%.6e" % fn])
print(json.dumps({"n_contacts": len(reporter.forces), "normal_force": sum(reporter.forces)}))
