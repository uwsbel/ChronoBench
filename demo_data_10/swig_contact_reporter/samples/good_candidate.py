"""A CORRECT turn-1 contact-reporter in a deliberately different style (different naming; reporter stores
(id, force) tuples). Same physics as the reference -> should PASS. Shows the judge does not penalize
stylistic divergence, only that the callback really enumerates contacts and reports true forces."""
import csv
import json
import math

import pychrono as chrono

n_spheres = 4
rad = 0.1
rho = 1.0 / ((4.0 / 3.0) * math.pi * rad ** 3)
grav = 9.81

world = chrono.ChSystemNSC()
world.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
world.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -grav, 0.0))

surf = chrono.ChContactMaterialNSC()
surf.SetFriction(0.4)
surf.SetRestitution(0.0)

floor = chrono.ChBodyEasyBox(4.0, 0.2, 4.0, 1000.0, True, True, surf)
floor.SetPos(chrono.ChVector3d(0.0, -0.1, 0.0))
floor.SetFixed(True)
world.AddBody(floor)

for k in range(n_spheres):
    ball = chrono.ChBodyEasySphere(rad, rho, True, True, surf)
    ball.SetPos(chrono.ChVector3d(-0.6 + 0.4 * k, rad + 0.05, 0.0))
    world.AddBody(ball)

for _ in range(1500):
    world.DoStepDynamics(1.0e-3)


class Reporter(chrono.ReportContactCallback):
    def __init__(self):
        super().__init__()
        self.rows = []

    def OnReportContact(self, pA, pB, plane_coord, distance, eff_radius, react_forces,
                        react_torques, modA, modB, contact_id):
        self.rows.append(abs(react_forces.x))
        return True


rep = Reporter()
world.GetContactContainer().ReportAllContacts(rep)

with open("out.csv", "w", newline="") as fh:
    wr = csv.writer(fh)
    wr.writerow(["contact_id", "normal_force"])
    for idx, fval in enumerate(rep.rows):
        wr.writerow([idx, "%.6e" % fval])

print(json.dumps({"n_contacts": len(rep.rows), "normal_force": sum(rep.rows)}))
