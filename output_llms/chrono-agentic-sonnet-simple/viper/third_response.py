import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# System setup (NSC — rovers use NSC)
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-down gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # small collision envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # small collision margin

# SCM deformable terrain (replaces rigid ground body)
terrain = veh.SCMTerrain(system)                                     # soft soil terrain
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.5))) # terrain plane at z=-0.5
terrain.Initialize(14, 4, 0.02)                                      # sizeX=14, sizeY=4, delta=0.02
terrain.SetSoilParameters(0.2e6, 0, 1.1, 0, 30, 0.01, 4e7, 3e4)   # Bekker/Janosi soil params
terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 20000)          # pressure color overlay
terrain.SetMeshWireframe(True)                                        # show wireframe mesh

# Viper rover setup
rover = robot.Viper(system)                                          # Viper owns its bodies
driver = robot.ViperDCMotorControl()                                 # DC motor steering driver
rover.SetDriver(driver)                                              # attach driver before Initialize

init_pos = chrono.ChVector3d(0, 0.2, 0)                             # initial position above terrain
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # identity rotation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))               # Initialize takes a ChFramed

# Irrlicht visualization (Initialize first, scene elements after)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up convention
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - SCM deformable terrain")
vis.Initialize()                                                      # FIRST — then scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, target
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                            # shadow-casting light

time_step = 1e-3                                                     # physics step size
sim_end = 20.0                                                       # simulation end time
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # render cadence (untagged)

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        driver.SetSteering(0.0)                                      # constant steering = 0.0 (straight)
        rover.Update()                                               # propagate steering to motors
        system.DoStepDynamics(time_step)                             # advance one step
        if system.GetChTime() >= sim_end:
            break
