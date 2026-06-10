import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 1e-3                                                      # physics step (1 ms → 1000 Hz)
render_fps = 50.0                                                     # 50 FPS as specified
render_step_size = 1.0 / render_fps                                   # time between rendered frames
sim_end = 200.0                                                       # run until stopped interactively

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # vehicle spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # facing +X

feda = veh.FEDA()                                                     # FEDA vehicle wrapper
feda.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
feda.SetChassisCollisionType(veh.CollisionType_NONE)                 # no chassis collision mesh
feda.SetChassisFixed(False)                                          # MANDATORY — free to move
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # spawn at origin, elevated
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)                   # simple map engine
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # automatic transmission
feda.SetTireType(veh.TireModelType_PAC02)                            # PAC02 tire model for FEDA
feda.SetTireStepSize(step_size)                                      # tire sub-step matches physics
feda.Initialize()                                                    # create the vehicle system

system = feda.GetSystem()                                            # get the owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", feda.GetVehicle().GetMass())                 # report total vehicle mass

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)         # mesh chassis
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # suspension
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # steering linkage
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)            # mesh wheels
feda.SetTireVisualizationType(veh.VisualizationType_MESH)             # mesh tires

terrain = veh.RigidTerrain(system)                                   # flat rigid terrain

patch_mat = chrono.ChContactMaterialNSC()                            # NSC material (matches FEDA truth)
patch_mat.SetFriction(0.9)                                           # high friction for driving
patch_mat.SetRestitution(0.01)                                       # nearly inelastic

patch = terrain.AddPatch(                                            # add flat terrain patch
    patch_mat,
    chrono.CSYSNORM,                                                 # centered at origin, Z-up
    200.0,                                                           # 200 m length (X)
    200.0                                                            # 200 m width (Y)
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # custom road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # sandy color

terrain.Initialize()                                                 # build terrain rigid bodies

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht vis
vis.SetWindowTitle("FEDA Vehicle Simulation")                        # window title
vis.SetWindowSize(1280, 1024)                                        # window resolution
vis.SetChaseCamera(                                                   # follow vehicle
    chrono.ChVector3d(0.0, 0.0, 1.75),                              # camera track point on chassis
    9.0,                                                             # distance behind vehicle
    0.5                                                              # height offset
)
vis.Initialize()                                                     # FIRST: create Irrlicht device
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # Chrono logo
vis.AddSkyBox()                                                      # sky background
vis.AddLightDirectional()                                            # directional light (vehicle truth style)
vis.AttachVehicle(feda.GetVehicle())                                 # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive keyboard driver

steering_time = 1.0                                                  # seconds to full steering deflection
throttle_time = 1.0                                                  # seconds to full throttle
braking_time = 0.3                                                   # seconds to full brake
driver.SetSteeringDelta(render_step_size / steering_time)            # per-render steering increment
driver.SetThrottleDelta(render_step_size / throttle_time)            # per-render throttle increment
driver.SetBrakingDelta(render_step_size / braking_time)              # per-render brake increment
driver.Initialize()                                                  # register with vis

render_every = max(1, round(render_step_size / step_size))           # physics steps between renders


realtime_timer = chrono.ChRealtimeStepTimer()                        # keeps wall-clock ≈ sim time
step_number = 0
while vis.Run():
    time = system.GetChTime()                                        # current sim time

    if step_number % render_every == 0:                              # throttled rendering at 50 FPS
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                               # fetch latest driver inputs

    driver.Synchronize(time)                                         # update driver state
    terrain.Synchronize(time)                                        # update terrain
    feda.Synchronize(time, driver_inputs, terrain)                   # advance vehicle subsystems
    vis.Synchronize(time, driver_inputs)                             # update vis HUD


    driver.Advance(step_size)                                        # step driver
    terrain.Advance(step_size)                                       # step terrain
    feda.Advance(step_size)                                          # step vehicle (advances ChSystem)
    vis.Advance(step_size)                                           # step vis

    step_number += 1
    realtime_timer.Spin(step_size)                                   # real-time pacing
