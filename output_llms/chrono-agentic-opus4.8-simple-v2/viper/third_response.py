import os
import pychrono as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

time_step = 1e-3                                                       # integration step (Viper uses 1e-3)
sim_end = 12.0                                                         # total simulated time (s)

system = chrono.ChSystemNSC()                                         # rovers use non-smooth contact
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Bullet collision (required for wheel<->soil contact)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-up world, g down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # small envelope for rover contact geometry
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # small margin

# SCM deformable (Bekker-Wong) terrain replacing the rigid ground.
terrain = veh.SCMTerrain(system)                                      # deformable soft-soil terrain on the shared system
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.5)))  # rest plane just below the rover spawn
terrain.SetSoilParameters(0.2e6,    # Bekker_Kphi   — frictional modulus (Pa)
                          0,        # Bekker_Kc     — cohesive modulus
                          1.1,      # Bekker_n      — sinkage exponent
                          0,        # Mohr_cohesion — cohesive limit (Pa)
                          30,       # Mohr_friction — friction angle (deg)
                          0.01,     # Janosi_shear  — shear deformation (m)
                          4e7,      # elastic_K     — elastic stiffness (Pa/m)
                          3e4)      # damping_R      — vertical damping (Pa.s/m)
terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 20000)          # color the sinkage/pressure overlay
terrain.SetMeshWireframe(True)                                       # show the deforming grid as wireframe
terrain.Initialize(14, 4, 0.02)                                      # sizeX, sizeY, grid resolution (m)

# Viper rover — owns its bodies; driver attached before Initialize.
rover = robot.Viper(system)                                          # 6-wheel suspension rover, adds its own bodies
driver = robot.ViperDCMotorControl()                                 # built-in DC-motor steering driver
rover.SetDriver(driver)                                              # attach driver BEFORE Initialize

init_pos = chrono.ChVector3d(0, 0, 0)                                # spawn at world origin, above the rest plane
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))               # Initialize takes a ChFramed

# Irrlicht visualization (Initialize first, then scene elements; NO grid).
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up camera
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - SCM deformable terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))   # eye, look-at
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                            # pos, aim, radius, near, far, angle, resolution

render_fps = 50.0                                                     # review-video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # physics steps between rendered frames

# Steering held constant at 0.0 — the rover drives straight; the DC drive is always on.
steering = 0.0                                                        # constant steering command (rad)
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        driver.SetSteering(steering)                                 # constant straight-line steering
        rover.Update()                                               # propagate command into the rover motors
        system.DoStepDynamics(time_step)                             # advance one physics step
        if system.GetChTime() >= sim_end:
            break
