"""SWIG contact reporter, turn 3 (EXTEND: heavier spheres + per-contact analysis) -- 10.0, headless, reference.

Same 6 resting spheres as turn 2, but each sphere's mass is DOUBLED to ~2 kg (density doubled). The contact
reporter logs each contact's normal force to out.csv. Beyond the aggregate, the judge now also inspects the
PER-CONTACT forces: at static equilibrium each of the 6 identical spheres presses on the ground with its own
weight, so every contact force ~= m*g ~= 19.62 N, and the summed force ~= 6*m*g ~= 117.7 N (independent-oracle
values). Tests that the callback's per-contact data (not just the total) is physically correct.
"""
import csv
import json
import math

import pychrono as chrono

N = 6
radius = 0.1
density = 2.0 / ((4.0 / 3.0) * math.pi * radius ** 3)   # turn-3 change: mass ~= 2.0 kg per sphere
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
    s.SetPos(chrono.ChVector3d(-1.0 + 0.4 * i, radius + 0.05, 0.0))
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
