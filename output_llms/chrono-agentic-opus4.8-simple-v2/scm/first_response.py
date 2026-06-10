import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(-5, 0, 0.6)                              # HMMWV spawn location above SCM rest plane
init_rot = chrono.QuatFromAngleZ(0)                                  # heading aligned with +X
step_size = 2e-3                                                      # vehicle dynamics step (s)
tire_step_size = 1e-3                                                 # tire force-model substep (s)
render_step_size = 1.0 / 50.0                                        # render at 50 fps (prompt)

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                   # SMC for SCM deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
hmmwv.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial pose
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                     # shaft-based engine model
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shaft transmission
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                         # all-wheel drive
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)               # pitman-arm steering
hmmwv.SetTireType(veh.TireModelType_RIGID)                          # prompt: rigid tire model
hmmwv.SetTireStepSize(tire_step_size)                              # tire integration step
hmmwv.Initialize()                                                  # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)    # mesh visualization on all components
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED before building SCMTerrain
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain_length = 100.0                                              # SCM patch X size (m)
terrain_width = 100.0                                               # SCM patch Y size (m)
terrain = veh.SCMTerrain(system)                                    # Bekker-Wong deformable soil on shared system
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi   — frictional modulus (Pa)
    0,      # Bekker_Kc     — cohesive modulus
    1.1,    # Bekker_n      — sinkage exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — internal friction angle (deg)
    0.01,   # Janosi_shear  — shear deformation modulus (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)
terrain.AddMovingPatch(                                             # moving patch follows the chassis
    hmmwv.GetChassisBody(),                                         # attach to chassis (stable OOBB), NOT spindles
    chrono.ChVector3d(0, 0, 0),                                    # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),                                    # OOBB dimensions (m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)        # false-color sinkage heatmap (0..0.1 m)
terrain.Initialize(terrain_length, terrain_width, 0.1)            # length, width, grid resolution (m)
terrain.SetMeshWireframe(False)                                  # solid deformable mesh
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 80, 80)  # soil texture tiling

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle-specific Irrlicht visual system
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)      # chase camera tracking the chassis
vis.Initialize()                                                 # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                        # vehicle demos use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                            # bind chassis/wheel/tire visual assets

driver = veh.ChInteractiveDriverIRR(vis)                         # interactive steering/throttle/braking driver
steering_time = 1.0                                              # s to ramp steering 0 -> 1
throttle_time = 1.0                                              # s to ramp throttle 0 -> 1
braking_time = 0.3                                               # s to ramp brake 0 -> 1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()                                             # arm the interactive driver

render_steps = math.ceil(render_step_size / step_size)          # physics steps between rendered frames
realtime_timer = chrono.ChRealtimeStepTimer()                   # spin to keep wall-clock == sim time
step_number = 0                                                 # physics step counter

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()                        # current simulation time

    if step_number % render_steps == 0:                        # throttled rendering at 50 fps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                         # collect steering/throttle/brake

    driver.Synchronize(time)                                   # update driver
    terrain.Synchronize(time)                                  # update terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)           # feed inputs + terrain to the vehicle
    vis.Synchronize(time, driver_inputs)                      # update HUD/view

    driver.Advance(step_size)                                 # advance driver
    terrain.Advance(step_size)                                # advance deformable soil
    hmmwv.Advance(step_size)                                  # advance wrapper-owned system
    vis.Advance(step_size)                                    # advance visualization


    step_number += 1                                          # advance step counter
    realtime_timer.Spin(step_size)                            # spin so wall-clock matches sim time
