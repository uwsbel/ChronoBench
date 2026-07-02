"""SWIG callback / lifecycle probe, turn 1 (CREATE) -- PyChrono 10.0, headless, reference.

Drops N=4 rigid spheres (mass ~1 kg each) onto a fixed plane, lets them settle, then a custom
Python-subclassed contact reporter (kept alive on the Python side) walks all contacts and LOGS each
contact's normal reaction force to out.csv. The judge derives the contact count and the summed normal
force from that CSV, so the callback must actually enumerate the contacts and report real forces
(un-gameable). Static equilibrium => count = N and sum of normal forces = N*m*g (independent-oracle
values). Probes: subclassing a C++ callback across SWIG, object lifecycle, and that the callback produced
data.
"""
import csv
import json
import math

import pychrono as chrono

N = 4
radius = 0.1
density = 1.0 / ((4.0 / 3.0) * math.pi * radius ** 3)   # -> mass ~= 1.0 kg per sphere
g = 9.81
t_end, dt = 1.5, 1.0e-3

sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # 10.0: collision system must be set
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -g, 0.0))

mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.4)
mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(4.0, 0.2, 4.0, 1000.0, True, True, mat)
ground.SetPos(chrono.ChVector3d(0.0, -0.1, 0.0))          # top face at y = 0
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
        self.forces.append(abs(react_forces.x))    # x is the contact-normal component
        return True


reporter = ContactReporter()                        # kept alive across the report call
sys.GetContactContainer().ReportAllContacts(reporter)

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["contact_id", "normal_force"])
    for i, fn in enumerate(reporter.forces):
        w.writerow([i, "%.6e" % fn])

print(json.dumps({"n_contacts": len(reporter.forces), "normal_force": sum(reporter.forces)}))
