import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemNSC()                                         # rovers use NSC
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # Bullet collision for wheel-terrain contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # g = 9.81, Z-down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # small contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # small contact margin

terrain = veh.SCMTerrain(system)                                     # deformable Bekker/Janosi soft soil
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.5)))  # SCM rest plane under the rover spawn
terrain.Initialize(14, 4, 0.02)                                      # sizeX, sizeY, grid delta (regular grid)
terrain.SetSoilParameters(0.2e6, 0, 1.1, 0, 30, 0.01, 4e7, 3e4)      # Kphi Kc n coh phi Janosi E damping
terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 20000)          # sinkage/pressure heatmap overlay
terrain.SetMeshWireframe(True)                                       # show deformation grid

rover = robot.Viper(system)                                          # built-in 6-wheel suspension rover
driver = robot.ViperDCMotorControl()                                 # DC-motor steering driver
rover.SetDriver(driver)                                              # attach driver BEFORE Initialize
init_pos = chrono.ChVector3d(0, 0, 0)                                # rover spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))               # Initialize takes a ChFramed

vis = chronoirr.ChVisualSystemIrrlicht()                            # Irrlicht render window
vis.AttachSystem(system)                                            # bind the system's visual assets
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                  # Z-up camera convention
vis.SetWindowSize(1280, 720)                                       # window resolution
vis.SetWindowTitle("Viper rover - SCM deformable terrain")        # window title
vis.Initialize()                                                   # create device FIRST, then scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                    # sky box
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, target
vis.AddTypicalLights()                                             # standard lighting
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                          # pos, aim, radius, near, far, angle, resolution

time_step = 1e-3                                                   # integration step
sim_end = 12.0                                                     # simulation duration (s)
render_fps = 50.0                                                  # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))      # untagged cadence constant
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        driver.SetSteering(0.0)                                   # constant straight-ahead steering
        rover.Update()                                           # propagate steering into the motors
        system.DoStepDynamics(time_step)                         # advance one physics step
        if system.GetChTime() >= sim_end:
            break
