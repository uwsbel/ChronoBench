import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC system, default solver
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up gravity, g = 9.81

ground = chrono.ChBody()                                             # fixed anchor for the spring
ground.SetFixed(True)                                                # ground does not move
ground.SetPos(chrono.ChVector3d(0, 0, 0))                            # anchor at the origin
sys.AddBody(ground)                                                  # register the ground

ground.AddVisualShape(chrono.ChVisualShapeBox(0.4, 0.1, 0.4),        # small slab marking the anchor
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

mass_box = chrono.ChBody()                                           # the suspended mass
mass_box.SetMass(1.0)                                                # mass = 1 kg
mass_box.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                    # diagonal inertia
mass_box.SetPos(chrono.ChVector3d(0, -2, 0))                         # hangs 2 m below the anchor
sys.AddBody(mass_box)                                                # register the mass

mass_box.AddVisualShape(chrono.ChVisualShapeBox(0.5, 0.5, 0.5),      # cube visual for the mass
                        chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

spring = chrono.ChLinkTSDA()                                         # translational spring-damper
spring.Initialize(mass_box, ground, True,                            # connect mass to ground (body-local frames)
                  chrono.ChVector3d(0, 0, 0),                        # attach point on the mass (its center)
                  chrono.ChVector3d(0, 0, 0))                        # attach point on the ground (origin)
spring.SetRestLength(2.0)                                            # natural length = initial gap
sys.AddLink(spring)                                                  # register the spring-damper

spring.AddVisualShape(chrono.ChVisualShapeSpring(0.1, 80, 15))       # coil visual: radius, resolution, turns

vis = chronoirr.ChVisualSystemIrrlicht()                            # Irrlicht render window
vis.AttachSystem(sys)                                                # bind the system to the visualizer
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                   # Y is the world up axis
vis.SetWindowSize(1280, 720)                                         # window resolution
vis.SetWindowTitle("Mass-Spring-Damper")                            # window title
vis.Initialize()                                                     # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo
vis.AddSkyBox()                                                      # standard sky box
vis.AddCamera(chrono.ChVector3d(0, -1, 6), chrono.ChVector3d(0, -1, 0))  # eye / look-at
vis.AddTypicalLights()                                               # standard lighting

time_step = 1e-3                                                     # integration step
sim_end = 10.0                                                       # simulation duration
render_fps = 50.0                                                    # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))        # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
