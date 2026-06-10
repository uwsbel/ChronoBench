import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(-5, 0, 0.6)                             # HMMWV spawn (above the bump)
init_rot = chrono.QuatFromAngleZ(0)                                  # facing +X
step_size = 2e-3                                                     # integration step (s)
tire_step_size = 1e-3                                                # tire substep for TMEASY on SCM

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                  # SMC for deformable SCM terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision geometry
hmmwv.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # SCM needs TMEASY (RIGID won't drive)
hmmwv.SetTireStepSize(tire_step_size)                              # tire force substep
hmmwv.Initialize()                                                 # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)   # mesh chassis
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                         # take ownership of the wrapper system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())             # report total vehicle mass

terrain = veh.SCMTerrain(system)                                   # deformable Bekker-Wong soil
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)        # color sinkage heatmap
terrain.AddMovingPatch(                                            # update only cells near the chassis
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),                                    # local OOBB centre
    chrono.ChVector3d(5, 3, 1),                                    # OOBB dims (m)
)
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),
                   40, 40, -1, 1, 0.02)                            # heightmap, len, wid, hMin, hMax, res
terrain.SetMeshWireframe(False)                                   # solid deformable mesh
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # soil texture

tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # tire radius
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()     # tire width
tire_mat = chrono.ChContactMaterialSMC()                          # SMC tire-soil material
tire_mat.SetFriction(0.9)                                         # high grip
tire_mat.SetRestitution(0.1)                                      # slight bounce

TIRE_FAMILY = 1                                                   # collision family for tires
for axle in hmmwv.GetVehicle().GetAxles():                       # add a collision cylinder per spindle
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()                 # spindle rigid body
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),  # +4cm to ensure sinkage
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)                            # enable SCM ray contact
        sp_cm = spindle.GetCollisionModel()                     # collision model handle
        sp_cm.SetFamily(TIRE_FAMILY)                            # tag as tire family
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)              # tires don't collide with each other
system.GetCollisionSystem().BindAll()                          # rebuild collision models after edits

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()              # vehicle-specific Irrlicht window
vis.SetWindowTitle("HMMWV on SCM Hill")                       # window title
vis.SetWindowSize(1280, 1024)                                # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)  # chase camera (track point, dist, height)
vis.Initialize()                                             # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # corner logo
vis.AddSkyBox()                                              # sky background
vis.AddLightDirectional()                                   # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                       # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                    # interactive keyboard driver (truth default)
render_step_size = 1.0 / 50.0                               # 50 FPS render cadence
steering_time = 1.0                                         # s to ramp steering 0 -> 1
throttle_time = 1.0                                         # s to ramp throttle 0 -> 1
braking_time = 0.3                                          # s to ramp brake 0 -> 1
driver.SetSteeringDelta(render_step_size / steering_time)  # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)  # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)    # braking rate
driver.Initialize()                                        # build the driver

render_steps = math.ceil(render_step_size / step_size)     # physics steps per rendered frame
sim_end = 20.0                                             # total simulated time (s)
render_every = max(1, render_steps)                        # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()           # spin to match wall clock
step_number = 0                                         # physics step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                          # current sim time

    if step_number % render_steps == 0:                # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                 # current driver command

    driver.Synchronize(time)                           # sync driver
    terrain.Synchronize(time)                          # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)    # sync vehicle (samples SCM via terrain)
    vis.Synchronize(time, driver_inputs)               # sync visuals

    driver.Advance(step_size)                          # advance driver
    terrain.Advance(step_size)                         # advance terrain
    hmmwv.Advance(step_size)                           # advances the wrapper-owned system
    vis.Advance(step_size)                             # advance visuals


    step_number += 1                                   # advance step counter
    realtime_timer.Spin(step_size)                     # spin in place to track wall clock
