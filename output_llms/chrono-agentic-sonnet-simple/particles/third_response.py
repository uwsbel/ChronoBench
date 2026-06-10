import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC for rigid-body dynamics
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # zero gravity — three-body inertial dynamics
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)       # required: spheres have collision shapes

# contact material for sphere-sphere collisions
mat = chrono.ChContactMaterialNSC()                                    # NSC material matching system type
mat.SetFriction(0.1)
mat.SetRestitution(0.9)                                                # highly elastic collisions

# sphere parameters
sph_radius = 1.5                                                       # sphere radius [m] — large enough to see clearly
sph_density = 1000.0                                                   # density [kg/m³]

# Sphere 1 — position (0, 0, 0), velocity (0.5, 0, 0.1)
sphere1 = chrono.ChBodyEasySphere(sph_radius, sph_density, True, True, mat)
sphere1.SetPos(chrono.ChVector3d(0, 0, 0))                             # sphere 1 at origin
sphere1.SetLinVel(chrono.ChVector3d(0.5, 0, 0.1))                     # initial velocity [m/s]
sphere1.GetVisualShape(0).SetColor(chrono.ChColor(1, 0.2, 0.2))       # red
sys.Add(sphere1)

# Sphere 2 — position (-10, -10, 0), velocity (-0.5, 0, -0.1)
sphere2 = chrono.ChBodyEasySphere(sph_radius, sph_density, True, True, mat)
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))                        # sphere 2 lower-left
sphere2.SetLinVel(chrono.ChVector3d(-0.5, 0, -0.1))                   # initial velocity [m/s]
sphere2.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 1, 0.2))       # green
sys.Add(sphere2)

# Sphere 3 — position (0, 20, 0), velocity (0, -0.5, 0.2)
sphere3 = chrono.ChBodyEasySphere(sph_radius, sph_density, True, True, mat)
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))                           # sphere 3 upper
sphere3.SetLinVel(chrono.ChVector3d(0, -0.5, 0.2))                    # initial velocity [m/s]
sphere3.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 1))       # blue
sys.Add(sphere3)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                      # Z-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Three-Body Particle Simulation")
vis.Initialize()                                                       # Initialize first (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(10, -30, 15), chrono.ChVector3d(0, 5, 0))  # view all three spheres
vis.AddTypicalLights()

time_step = 1e-3                                                       # physics step [s]
sim_end = 20.0                                                         # simulation duration [s]
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))           # render cadence constant

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
