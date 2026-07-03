"""A CORRECT-BUT-DIFFERENT turn-1 bouncing ball: same physics, different style. Builds plain
ChBody objects with explicit collision shapes (ChCollisionShapeBox / ChCollisionShapeSphere)
instead of the ChBodyEasy helpers, sets the sphere's mass and inertia by hand, and logs every 10th
step (a 2e-3 s logging cadence; the apex is still resolved to sub-millimeter). Should pass
L1/L2/L3 near ceiling."""
import csv
import json
import math

import pychrono.core as chrono

E_REST = 0.7
H0 = 1.0
RADIUS = 0.1
DENSITY = 1000.0
DT = 2.0e-4
T_END = 1.5

system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

material = chrono.ChContactMaterialNSC()
material.SetRestitution(E_REST)
material.SetFriction(0.3)

ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, -0.1, 0))
ground.AddCollisionShape(chrono.ChCollisionShapeBox(material, 2.0, 0.2, 2.0))
ground.EnableCollision(True)
system.Add(ground)

mass = DENSITY * 4.0 / 3.0 * math.pi * RADIUS ** 3
inertia = 0.4 * mass * RADIUS ** 2
sphere = chrono.ChBody()
sphere.SetPos(chrono.ChVector3d(0, H0 + RADIUS, 0))
sphere.SetMass(mass)
sphere.SetInertiaXX(chrono.ChVector3d(inertia, inertia, inertia))
sphere.AddCollisionShape(chrono.ChCollisionShapeSphere(material, RADIUS))
sphere.EnableCollision(True)
system.Add(sphere)

rows = []
step = 0
apex = -1.0
while system.GetChTime() < T_END:
    system.DoStepDynamics(DT)
    step += 1
    t = system.GetChTime()
    y_bot = sphere.GetPos().y - RADIUS
    if step % 10 == 0:
        rows.append((t, y_bot))
    if t >= 0.5 and y_bot > apex:
        apex = y_bot

with open("out.csv", "w", newline="") as fh:
    writer = csv.writer(fh)
    writer.writerow(["t", "y_bot"])
    for t, y in rows:
        writer.writerow([f"{t:.6f}", f"{y:.6e}"])

print(json.dumps({"apex1": apex, "restitution": E_REST, "method": "NSC"}))
