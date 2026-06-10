import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 2e-3                                                      # integration step
tire_step_size = 1e-3                                                 # tire substep
init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn (origin of cross roads)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # identity orientation

hmmwv = veh.HMMWV_Full()                                             # full HMMWV model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision shape
hmmwv.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # place the vehicle
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire on rigid road
hmmwv.SetTireStepSize(tire_step_size)                              # tire integration substep
hmmwv.Initialize()                                                  # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)   # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)     # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)      # tire mesh

system = hmmwv.GetSystem()                                          # wrapper owns the system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # rigid terrain on the vehicle system

patch_mat = chrono.ChContactMaterialNSC()                          # NSC patch material
patch_mat.SetFriction(0.4)                                          # friction coefficient
patch_mat.SetRestitution(0.05)                                      # restitution (bounciness)

patch_csys = chrono.ChCoordsysd(                                    # patch pose: rotated/repositioned road
    chrono.ChVector3d(6, -70, 0),                                  # patch position at the cross roads
    chrono.QuatFromAngleZ(-math.pi / 2.0),                         # -90 degrees about Z axis
)
patch = terrain.AddPatch(patch_mat, patch_csys, 200.0, 200.0)     # flat patch, 200 x 200 m

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # patch tint

terrain.Initialize()                                               # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle Irrlicht window
vis.SetWindowTitle("Rigid Highway")                              # window title
vis.SetWindowSize(1280, 1024)                                    # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)     # chase camera behind chassis
vis.Initialize()                                                 # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo overlay
vis.AddSkyBox()                                                  # sky box
vis.AddLightDirectional()                                        # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())                            # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                         # interactive keyboard driver (truth shape)
render_step_size = 1.0 / 50.0                                    # 50 fps render cadence
steering_time = 1.0                                              # seconds 0 -> +1 steering
throttle_time = 1.0                                              # seconds 0 -> +1 throttle
braking_time = 0.3                                               # seconds 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)        # steering ramp
driver.SetThrottleDelta(render_step_size / throttle_time)        # throttle ramp
driver.SetBrakingDelta(render_step_size / braking_time)          # brake ramp
driver.Initialize()                                              # finalize driver

render_steps = math.ceil(render_step_size / step_size)          # physics steps per rendered frame
sim_end = 12.0                                                   # simulation duration (s)

realtime_timer = chrono.ChRealtimeStepTimer()                   # wall-clock pacing
step_number = 0                                                 # step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                  # current sim time

    if step_number % render_steps == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                         # current driver inputs

    driver.Synchronize(time)                                   # sync driver
    terrain.Synchronize(time)                                  # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)           # sync vehicle with inputs + terrain
    vis.Synchronize(time, driver_inputs)                      # sync vis HUD

    driver.Advance(step_size)                                  # advance driver
    terrain.Advance(step_size)                                 # advance terrain
    hmmwv.Advance(step_size)                                   # advance vehicle (steps the system)
    vis.Advance(step_size)                                     # advance vis


    step_number += 1                                           # advance step count
    realtime_timer.Spin(step_size)                            # spin so wall-clock matches sim time
