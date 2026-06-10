import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# system (NSC — no contact needed for pure spring-mass)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))    # Y-up gravity

# ground body — fixed reference
ground = chrono.ChBody()
ground.SetFixed(True)                                                # immovable ground
ground.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(ground)

ground_box = chrono.ChVisualShapeBox(0.4, 0.1, 0.4)                 # small block for anchor viz
ground_box.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground.AddVisualShape(ground_box, chrono.ChFramed(chrono.ChVector3d(0, 0.05, 0)))

# mass body — 1 kg block hanging below ground attachment point
mass = chrono.ChBody()
mass.SetMass(1.0)                                                    # 1 kg mass
mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))              # small inertia tensor
mass.SetPos(chrono.ChVector3d(0, -2.0, 0))                          # 2 m below ground
sys.AddBody(mass)

mass_box = chrono.ChVisualShapeBox(0.4, 0.4, 0.4)                   # visible mass cube
mass_box.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
mass.AddVisualShape(mass_box)

# spring-damper link (ChLinkTSDA) between ground and mass
spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, True,
                  chrono.ChVector3d(0, 0, 0),                        # attachment point on ground (body-local)
                  chrono.ChVector3d(0, 0, 0))                        # attachment point on mass (body-local)
spring.SetRestLength(1.5)                                            # rest length 1.5 m
spring.SetSpringCoefficient(50.0)                                    # spring stiffness k = 50 N/m
spring.SetDampingCoefficient(5.0)                                    # damping c = 5 N·s/m
sys.AddLink(spring)

spring_vis = chrono.ChVisualShapeSpring(0.05, 80, 15)               # coil radius, resolution, turns
spring_vis.SetColor(chrono.ChColor(0.8, 0.4, 0.1))
spring.AddVisualShape(spring_vis)                                    # visual spring attached to link

# Irrlicht visualization — Initialize() FIRST, then scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                    # Y-up scene
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, -2, 3), chrono.ChVector3d(0, -1, 0))  # view from side
vis.AddTypicalLights()

# time stepping parameters
time_step = 1e-3                                                     # 1 ms step
sim_end = 10.0                                                       # 10 s simulation
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # physics steps per frame


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
