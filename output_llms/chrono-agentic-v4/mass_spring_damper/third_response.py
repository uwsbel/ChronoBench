"""
Mass-Spring-Damper chain: ground --spring1--> body_1 --spring2--> body_2 --spring3--> body_3.
Three bodies connected in series by springs with one end fixed to ground.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Named constants ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Body masses [kg]
m1 = 1.0
m2 = 1.0
m3 = 1.0

# Spring constants [N/m]
k1 = 100.0
k2 = 100.0
k3 = 100.0

# Damping coefficients [N*s/m]
c1 = 2.0
c2 = 2.0
c3 = 2.0

# Rest lengths [m]
rest1 = 1.0
rest2 = 1.0
rest3 = 1.0

# Body geometry [m]
body_size = 0.2

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Collision material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)
mat.SetRestitution(0.0)

# === Ground ===
ground = chrono.ChBodyEasyBox(4.0, 0.1, 4.0, 1000.0, True, True, mat)
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(1.5, -0.05, 0.0))
sys.AddBody(ground)

# === Bodies ===
# body_1: cube at x = rest1
body1 = chrono.ChBody()
body1.SetMass(m1)
body1.SetInertiaXX(chrono.ChVector3d(
    (1.0 / 6.0) * m1 * body_size * body_size,
    (1.0 / 6.0) * m1 * body_size * body_size,
    (1.0 / 6.0) * m1 * body_size * body_size,
))
body1.SetPos(chrono.ChVector3d(rest1, body_size / 2.0 + 0.05, 0.0))
body1.EnableCollision(True)
sys.AddBody(body1)
box1 = chrono.ChVisualShapeBox(body_size, body_size, body_size)
box1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
body1.AddVisualShape(box1)

# body_2: cube at x = rest1 + rest2
body2 = chrono.ChBody()
body2.SetMass(m2)
body2.SetInertiaXX(chrono.ChVector3d(
    (1.0 / 6.0) * m2 * body_size * body_size,
    (1.0 / 6.0) * m2 * body_size * body_size,
    (1.0 / 6.0) * m2 * body_size * body_size,
))
body2.SetPos(chrono.ChVector3d(rest1 + rest2, body_size / 2.0 + 0.05, 0.0))
body2.EnableCollision(True)
sys.AddBody(body2)
box2 = chrono.ChVisualShapeBox(body_size, body_size, body_size)
box2.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
body2.AddVisualShape(box2)

# body_3: cube at x = rest1 + rest2 + rest3
body3 = chrono.ChBody()
body3.SetMass(m3)
body3.SetInertiaXX(chrono.ChVector3d(
    (1.0 / 6.0) * m3 * body_size * body_size,
    (1.0 / 6.0) * m3 * body_size * body_size,
    (1.0 / 6.0) * m3 * body_size * body_size,
))
body3.SetPos(chrono.ChVector3d(rest1 + rest2 + rest3, body_size / 2.0 + 0.05, 0.0))
body3.EnableCollision(True)
sys.AddBody(body3)
box3 = chrono.ChVisualShapeBox(body_size, body_size, body_size)
box3.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
body3.AddVisualShape(box3)

# === Springs ===
# Spring 1: ground -> body_1
spring1 = chrono.ChLinkTSDA()
spring1.Initialize(ground, body1, True,
                    chrono.ChVector3d(0.0, 0.0, 0.0),
                    chrono.ChVector3d(0.0, 0.0, 0.0))
spring1.SetRestLength(rest1)
spring1.SetSpringCoefficient(k1)
spring1.SetDampingCoefficient(c1)
spring1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 12, 8))
sys.AddLink(spring1)

# Spring 2: body_1 -> body_2
spring2 = chrono.ChLinkTSDA()
spring2.Initialize(body1, body2, True,
                  chrono.ChVector3d(0.0, 0.0, 0.0),
                  chrono.ChVector3d(0.0, 0.0, 0.0))
spring2.SetRestLength(rest2)
spring2.SetSpringCoefficient(k2)
spring2.SetDampingCoefficient(c2)
spring2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 12, 8))
sys.AddLink(spring2)

# Spring 3: body_2 -> body_3
spring3 = chrono.ChLinkTSDA()
spring3.Initialize(body2, body3, True,
                   chrono.ChVector3d(0.0, 0.0, 0.0),
                   chrono.ChVector3d(0.0, 0.0, 0.0))
spring3.SetRestLength(rest3)
spring3.SetSpringCoefficient(k3)
spring3.SetDampingCoefficient(c3)
spring3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 12, 8))
sys.AddLink(spring3)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper Chain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, -0.5, 2.5), chrono.ChVector3d(1.5, 0.1, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))


frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
    frame += 1
