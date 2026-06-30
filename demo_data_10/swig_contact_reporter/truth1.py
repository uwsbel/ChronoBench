"""SWIG callback / lifecycle probe (PyChrono 10.0, headless) -- contracted reference.

Drops N rigid spheres (each mass ~1 kg) onto a fixed plane, lets them settle, then a custom
Python-subclassed contact reporter (kept alive on the Python side) walks all contacts and accumulates
the count and the summed normal reaction. Probes: subclassing a C++ callback across SWIG, object
lifecycle (no GC of the live callback), and that the callback actually produced data.
Invariants: n_contacts >= N (callback invoked + produced data); summed normal force ~= N*m*g
(static equilibrium of the resting spheres).
"""
import math
import json

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
ground.SetPos(chrono.ChVector3d(0.0, -0.1, 0.0))      # top face at y = 0
ground.SetFixed(True)
sys.AddBody(ground)

for i in range(N):
    s = chrono.ChBodyEasySphere(radius, density, True, True, mat)
    s.SetPos(chrono.ChVector3d(-0.6 + 0.4 * i, radius + 0.05, 0.0))   # spaced in x; small drop
    sys.AddBody(s)

while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)


class ContactReporter(chrono.ReportContactCallback):
    def __init__(self):
        super().__init__()
        self.n = 0
        self.normal_force = 0.0

    def OnReportContact(self, pA, pB, plane_coord, distance, eff_radius, react_forces,
                        react_torques, modA, modB, contact_id):
        self.n += 1
        self.normal_force += abs(react_forces.x)    # x is the contact-normal component
        return True


reporter = ContactReporter()                          # kept alive across the report call
sys.GetContactContainer().ReportAllContacts(reporter)

print(json.dumps({"n_contacts": reporter.n, "normal_force": reporter.normal_force}))
