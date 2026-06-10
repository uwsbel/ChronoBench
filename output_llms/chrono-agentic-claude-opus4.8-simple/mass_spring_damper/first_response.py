import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                          # rigid-body MBS system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))        # gravity OFF — spring is the only force

rest_length = 1.5                                                   # spring rest length

ground = chrono.ChBody()                                            # fixed anchor for the spring
ground.SetFixed(True)                                              # ground does not move
ground.EnableCollision(False)                                      # no contact in this scene
sys.AddBody(ground)
ground.AddVisualShape(chrono.ChVisualShapeSphere(0.1),             # small sphere marks the anchor
                      chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

body_1 = chrono.ChBody()                                            # the hanging mass
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))                        # starts below the anchor
body_1.SetMass(1)                                                  # 1 kg
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                   # diagonal inertia
body_1.EnableCollision(False)                                     # no contact
sys.AddBody(body_1)
box_1 = chrono.ChVisualShapeBox(1, 1, 1)                          # unit cube for the mass
box_1.SetColor(chrono.ChColor(0.6, 0, 0))                        # dark red
body_1.AddVisualShape(box_1)

spring_1 = chrono.ChLinkTSDA()                                     # translational spring-damper
spring_1.Initialize(body_1, ground, True,                         # True = endpoints in body-relative frames
                    chrono.ChVector3d(0, 0, 0),                   # attach at body_1 origin
                    chrono.ChVector3d(-1, 0, 0))                  # attach at the ground anchor
spring_1.SetRestLength(rest_length)                               # default linear spring-damper params otherwise
sys.AddLink(spring_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15)) # coil radius, resolution, turns

vis = chronoirr.ChVisualSystemIrrlicht()                          # Irrlicht render window
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)                                      # window resolution
vis.SetWindowTitle("ChLinkTSDA demo")
vis.Initialize()                                                 # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()                                                  # standard sky box
vis.AddCamera(chrono.ChVector3d(0, 0, 6))                       # look toward the spring axis
vis.AddTypicalLights()                                          # standard two-light setup

time_step = 1e-3                                                # integration step
render_fps = 50.0                                              # review playback rate
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
