import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # no global gravity; mutual attraction only
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # collision system for the spheres

sph_radius = 1.0                                                      # sphere radius [m]
sph_density = 1000.0                                                  # sphere density [kg/m^3]

mat = chrono.ChContactMaterialNSC()                                   # NSC contact material for the spheres

sphere1 = chrono.ChBodyEasySphere(sph_radius, sph_density, True, True, mat)  # visualize, collide
sphere1.SetPos(chrono.ChVector3d(10, 10, 0))                         # sphere 1 initial position
sphere1.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))                     # sphere 1 initial velocity
sphere1.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.2, 0.2))    # red
sys.Add(sphere1)                                                     # add sphere 1

sphere2 = chrono.ChBodyEasySphere(sph_radius, sph_density, True, True, mat)  # visualize, collide
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))                       # sphere 2 initial position
sphere2.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1))                   # sphere 2 initial velocity
sphere2.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.9, 0.2))    # green
sys.Add(sphere2)                                                     # add sphere 2

sphere3 = chrono.ChBodyEasySphere(sph_radius, sph_density, True, True, mat)  # visualize, collide
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))                          # sphere 3 initial position
sphere3.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2))                    # sphere 3 initial velocity
sphere3.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.9))    # blue
sys.Add(sphere3)                                                     # add sphere 3

bodies = [sphere1, sphere2, sphere3]                                 # the three interacting bodies
G = 6.674e-11                                                        # gravitational constant
G *= 1e9                                                             # scale up so the masses produce visible attraction

vis = chronoirr.ChVisualSystemIrrlicht()                            # Irrlicht render window
vis.AttachSystem(sys)                                               # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                   # Z-up camera
vis.SetWindowSize(1280, 720)                                        # window size
vis.SetWindowTitle("Three-Body Sphere Simulation")                 # window title
vis.Initialize()                                                   # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                                    # sky box
vis.AddCamera(chrono.ChVector3d(0, -60, 30), chrono.ChVector3d(0, 0, 0))  # eye, look-at origin
vis.AddTypicalLights()                                             # standard lights

time_step = 1e-3                                                    # integration step [s]
sim_end = 20.0                                                      # simulation duration [s]
render_fps = 50.0                                                   # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))       # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                               # begin frame
    vis.Render()                                                   # draw scene
    vis.EndScene()                                                 # present frame
    for _ in range(render_every):
        for b in bodies:                                           # reset accumulated forces each step
            b.EmptyAccumulators()
        for i in range(len(bodies)):                               # pairwise Newtonian attraction
            for j in range(i + 1, len(bodies)):
                d = bodies[j].GetPos() - bodies[i].GetPos()        # vector from i to j
                r = d.Length()                                     # separation distance
                if r > 2 * sph_radius:                             # skip when overlapping/contacting
                    f = G * bodies[i].GetMass() * bodies[j].GetMass() / (r * r)  # force magnitude
                    fvec = d * (f / r)                             # force on i toward j
                    bodies[i].AccumulateForce(fvec, bodies[i].GetPos(), False)   # pull i toward j
                    bodies[j].AccumulateForce(-fvec, bodies[j].GetPos(), False)  # pull j toward i
        sys.DoStepDynamics(time_step)                              # advance one step
        if sys.GetChTime() >= sim_end:
            break
