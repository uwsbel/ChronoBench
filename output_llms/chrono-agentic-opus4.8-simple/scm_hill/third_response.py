import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                    # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # locate vehicle data files

init_loc = chrono.ChVector3d(-5, 0, 0.6)                                # vehicle spawn (heightmap centered at origin)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # identity orientation
step_size = 2e-3                                                        # integration step

hmmwv = veh.HMMWV_Full()                                                # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                      # NSC contact (rigid terrain)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                  # no chassis collision shape
hmmwv.SetChassisFixed(False)                                           # chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                            # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(step_size)                                       # tire integration step
hmmwv.Initialize()                                                     # build the vehicle subsystems
system = hmmwv.GetSystem()                                             # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # required for contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                  # report total vehicle mass

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)         # chassis mesh visuals
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)           # wheel mesh visuals
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)            # tire mesh visuals

terrain = veh.RigidTerrain(system)                                     # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                             # NSC contact material
patch_mat.SetFriction(0.9)                                            # terrain friction
patch_mat.SetRestitution(0.01)                                        # terrain restitution
patch = terrain.AddPatch(                                              # single heightmap patch
    patch_mat,
    chrono.CSYSNORM,                                                  # centered at origin, no rotation
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),               # height map bitmap
    64, 64,                                                           # patch length / width (m)
    0.0, 1.0,                                                         # min / max height (m)
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture
terrain.Initialize()                                                  # build terrain collision + visuals

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                       # vehicle Irrlicht window
vis.SetWindowTitle("HMMWV on rigid heightmap terrain")                # window title
vis.SetWindowSize(1280, 1024)                                         # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)          # chase camera over chassis
vis.Initialize()                                                     # build the device (call FIRST)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))      # logo overlay
vis.AddSkyBox()                                                       # sky box
vis.AddLightDirectional()                                            # directional light (vehicle truth)
vis.AttachVehicle(hmmwv.GetVehicle())                                # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                              # interactive driver bound to vis
render_step_size = 1.0 / 50.0                                        # render cadence (s)
steering_time = 1.0                                                  # 0 -> 1 steering time
throttle_time = 1.0                                                  # 0 -> 1 throttle time
braking_time = 0.3                                                   # 0 -> 1 brake time
driver.SetSteeringDelta(render_step_size / steering_time)            # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)            # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)              # brake rate
driver.Initialize()                                                 # initialize driver

sim_end = 12.0                                                       # simulation duration (s)
render_steps = math.ceil(render_step_size / step_size)              # physics steps per frame
render_every = max(1, render_steps)                                 # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
step_number = 0                                                     # physics step counter
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()                                  # current sim time
        driver_inputs = driver.GetInputs()                         # poll driver inputs

        driver.Synchronize(time)                                   # sync driver
        terrain.Synchronize(time)                                  # sync terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)           # sync vehicle with terrain
        vis.Synchronize(time, driver_inputs)                      # sync visualization

        driver.Advance(step_size)                                  # advance driver
        terrain.Advance(step_size)                                 # advance terrain
        hmmwv.Advance(step_size)                                   # advance vehicle (steps the system)
        vis.Advance(step_size)                                     # advance visualization

        step_number += 1                                           # advance step counter
        realtime_timer.Spin(step_size)                            # spin to match wall-clock
        if system.GetChTime() >= sim_end:
            break
