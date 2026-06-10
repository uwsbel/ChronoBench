import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC system for the three-body problem
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # no uniform field; gravity is body-to-body only

grav_const = 6.674e-11                                                # Newtonian gravitational constant G [m^3 kg^-1 s^-2]
sph_radius = 1.0                                                      # sphere radius [m]
sph_density = 5.0e10                                                  # high density so mutual gravity bends the trajectories visibly

# Sphere 1: radius, density, visualize, collide -> dynamic body
sphere1 = chrono.ChBodyEasySphere(sph_radius, sph_density, True, False)  # visualize, no collide (pure gravity N-body)
sphere1.SetPos(chrono.ChVector3d(10, 10, 0))                          # initial position of sphere 1
sphere1.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))                      # initial velocity of sphere 1
sphere1.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.2, 0.2))    # red
sys.Add(sphere1)                                                      # register sphere 1

# Sphere 2 at (-10, -10, 0)
sphere2 = chrono.ChBodyEasySphere(sph_radius, sph_density, True, False)  # visualize, no collide
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))                       # initial position of sphere 2
sphere2.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1))                   # initial velocity of sphere 2
sphere2.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.9, 0.2))   # green
sys.Add(sphere2)                                                      # register sphere 2

# Sphere 3 at (0, 20, 0)
sphere3 = chrono.ChBodyEasySphere(sph_radius, sph_density, True, False)  # visualize, no collide
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))                          # initial position of sphere 3
sphere3.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2))                    # initial velocity of sphere 3
sphere3.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.9))   # blue
sys.Add(sphere3)                                                      # register sphere 3

bodies = [sphere1, sphere2, sphere3]                                  # the three interacting masses

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht render window
vis.AttachSystem(sys)                                                 # bind the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up camera
vis.SetWindowSize(1280, 720)                                         # window pixels
vis.SetWindowTitle("Three-Body Gravitational Simulation")           # window title
vis.Initialize()                                                     # create device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, -45, 20), chrono.ChVector3d(0, 0, 0))  # eye, look-at origin
vis.AddTypicalLights()                                              # standard lights

time_step = 1e-3                                                     # integration step [s]
sim_end = 30.0                                                       # total simulated time [s]
render_fps = 50.0                                                   # frames per second for the review video
render_every = max(1, round(1.0 / (render_fps * time_step)))        # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        for b in bodies:
            b.EmptyAccumulators()                                   # clear last step's accumulated force
        for i in range(len(bodies)):                                # pairwise Newtonian attraction
            for j in range(i + 1, len(bodies)):
                bi = bodies[i]; bj = bodies[j]
                d = bj.GetPos() - bi.GetPos()                       # vector from i to j
                dist = d.Length()                                  # separation distance [m]
                if dist > 1e-6:
                    f_mag = grav_const * bi.GetMass() * bj.GetMass() / (dist * dist)  # |F| = G m_i m_j / r^2
                    f_dir = d * (1.0 / dist)                        # unit vector i -> j
                    force = f_dir * f_mag                           # attractive force on i toward j
                    bi.AccumulateForce(force, bi.GetPos(), False)   # pull i toward j (world frame)
                    bj.AccumulateForce(-force, bj.GetPos(), False)  # equal-and-opposite pull on j
        sys.DoStepDynamics(time_step)                              # advance one physics step
        if sys.GetChTime() >= sim_end:
            break
