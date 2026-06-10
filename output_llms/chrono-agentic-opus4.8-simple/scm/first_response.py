import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(-5, 0, 0.6)                             # vehicle spawn location (X, Y, Z)
init_rot = chrono.QuatFromAngleZ(0)                                  # heading: facing +X
step_size = 2e-3                                                     # integration step (s)
tire_step_size = 1e-3                                                # tire model step (s)

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                  # SMC for SCM deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision geometry
hmmwv.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # location + orientation
hmmwv.SetTireType(veh.TireModelType_RIGID)                          # prompt: rigid tire model
hmmwv.SetTireStepSize(tire_step_size)                              # tire model step size
hmmwv.Initialize()                                                  # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)   # mesh on all vehicle components
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize, before SCM
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.SCMTerrain(system)                                    # deformable Bekker-Wong soft soil
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa.s/m)
)
terrain.AddMovingPatch(                                             # patch follows the chassis (NOT spindles)
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),                                     # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),                                     # OOBB dimensions (m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)        # false-color sinkage plotting
terrain.Initialize(40.0, 40.0, 0.1)                                # length (m), width (m), grid resolution (m)
terrain.SetMeshWireframe(False)                                    # solid deformable mesh
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80)  # soil texture

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-specific Irrlicht system
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")             # window title
vis.SetWindowSize(1280, 1024)                                      # window size (px)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)      # chase camera on chassis
vis.Initialize()                                                  # build device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo after Initialize
vis.AddSkyBox()                                                   # sky box after Initialize
vis.AddLightDirectional()                                         # vehicle scenes use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                            # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive steering/throttle/brake
render_step_size = 1.0 / 50.0                                     # render at 50 fps
steering_time = 1.0                                               # s to go 0 -> +1 steering
throttle_time = 1.0                                               # s to go 0 -> +1 throttle
braking_time = 0.3                                                # s to go 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)        # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)        # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)          # brake rate
driver.Initialize()                                              # finalize the driver

render_steps = math.ceil(render_step_size / step_size)           # physics steps per rendered frame (untagged cadence)

realtime_timer = chrono.ChRealtimeStepTimer()                   # spin so wall-clock matches sim time
step_number = 0                                                 # physics step counter
while vis.Run():
    time = hmmwv.GetSystem().GetChTime()                        # current sim time

    if step_number % render_steps == 0:                         # throttled rendering at 50 fps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                          # interactive driver inputs

    driver.Synchronize(time)                                    # sync driver
    terrain.Synchronize(time)                                   # sync deformable terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)            # sync vehicle against terrain
    vis.Synchronize(time, driver_inputs)                       # sync visualization + HUD


    driver.Advance(step_size)                                   # advance driver
    terrain.Advance(step_size)                                  # advance terrain deformation
    hmmwv.Advance(step_size)                                    # advances the wrapper-owned system
    vis.Advance(step_size)                                      # advance visualization

    step_number += 1                                            # next step
    realtime_timer.Spin(step_size)                             # spin in place for real time
