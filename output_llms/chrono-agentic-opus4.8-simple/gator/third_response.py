import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis-origin spawn (above terrain)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation

step_size = 2e-3                                                     # integration step
tire_step_size = 1e-3                                                # tire substep

gator = veh.Gator()                                                 # Gator catalog wrapper (owns its system)
gator.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)               # add primitive chassis collision manually below
gator.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # initial pose
gator.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY tires on rigid terrain
gator.SetTireStepSize(tire_step_size)                               # tire substep size
gator.Initialize()                                                  # build the vehicle subsystems

gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)     # prompt: simplify mesh -> primitives
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # primitives
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # primitives
gator.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)       # primitives
gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)        # primitives

system = gator.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())               # report total vehicle mass

chassis_body = gator.GetChassisBody()                               # chassis rigid body
cmat = chrono.ChContactMaterialNSC()                                # NSC contact material for the chassis box
cmat.SetFriction(0.7)                                               # chassis friction
chassis_body.AddCollisionShape(                                     # prompt: add chassis collision as a primitive box
    chrono.ChCollisionShapeBox(cmat, 2.0, 1.0, 0.4),               # simple box approximating the chassis hull
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT),   # raised into the upper chassis volume, clear of wheels
)
chassis_body.EnableCollision(True)                                  # enable chassis collision

terrain = veh.RigidTerrain(system)                                  # flat rigid terrain on the shared system
patch_mat = chrono.ChContactMaterialNSC()                           # terrain contact material
patch_mat.SetFriction(0.9)                                          # terrain friction
patch_mat.SetRestitution(0.01)                                      # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100.0, 100.0)  # 100x100 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # terrain texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # terrain color
terrain.Initialize()                                               # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle-specific Irrlicht visual system
vis.SetWindowTitle("Gator Vehicle")                                 # window title (before Initialize)
vis.SetWindowSize(1280, 1024)                                       # window size (before Initialize)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)        # chase camera point/distance/height
vis.Initialize()                                                   # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo (after Initialize)
vis.AddSkyBox()                                                     # sky box (after Initialize)
vis.AddLightDirectional()                                          # vehicle demos use a directional light
vis.AttachVehicle(gator.GetVehicle())                              # bind vehicle visual assets

render_step_size = 1.0 / 50.0                                       # render once per 1/50 s
render_steps = math.ceil(render_step_size / step_size)             # physics steps per rendered frame

driver = veh.ChInteractiveDriverIRR(vis)                           # interactive keyboard driver bound to the vis
steering_time = 2.0                                                 # prompt: slower controls -> longer 0->1 steering time
throttle_time = 2.0                                                 # prompt: slower controls -> longer 0->1 throttle time
braking_time = 1.0                                                  # prompt: slower controls -> longer 0->1 braking time
driver.SetSteeringDelta(render_step_size / steering_time)          # smaller per-frame delta = less responsive
driver.SetThrottleDelta(render_step_size / throttle_time)          # smaller per-frame delta = less responsive
driver.SetBrakingDelta(render_step_size / braking_time)            # smaller per-frame delta = less responsive
driver.Initialize()                                                # finalize the driver

system.GetCollisionSystem().BindAll()                              # bind after the manual chassis collision edit

render_every = render_steps                                         # untagged render cadence

realtime_timer = chrono.ChRealtimeStepTimer()                      # real-time pacing
step_number = 0                                                     # physics step counter
while vis.Run():
    time = system.GetChTime()                                      # current sim time

    if step_number % render_every == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                            # current driver inputs

    driver.Synchronize(time)                                      # sync driver
    terrain.Synchronize(time)                                     # sync terrain
    gator.Synchronize(time, driver_inputs, terrain)              # sync vehicle with inputs + terrain
    vis.Synchronize(time, driver_inputs)                         # sync visualization HUD


    driver.Advance(step_size)                                     # advance driver
    terrain.Advance(step_size)                                    # advance terrain
    gator.Advance(step_size)                                      # advance vehicle (steps the owned system)
    vis.Advance(step_size)                                        # advance visualization

    step_number += 1                                              # next step
    realtime_timer.Spin(step_size)                               # spin so wall-clock matches sim time
