import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # vehicle spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # no initial heading rotation
step_size = 2e-3                                                     # integration step (s)
tire_step_size = 1e-3                                                # tire substep (s)

vis_type = veh.VisualizationType_PRIMITIVES                         # primitives (simplified from mesh)
terrainLength = 100.0                                                # terrain X size (m)
terrainWidth = 100.0                                                 # terrain Y size (m)

gator = veh.Gator()                                                  # catalog Gator vehicle wrapper
gator.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)              # add primitive collision manually below
gator.SetChassisFixed(False)                                       # MANDATORY — fixed chassis won't move
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn pose
gator.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire on rigid terrain
gator.SetTireStepSize(tire_step_size)                              # tire integration substep
gator.Initialize()                                                 # build the vehicle subsystems

gator.SetChassisVisualizationType(vis_type)                        # primitives for chassis
gator.SetSuspensionVisualizationType(vis_type)                     # primitives for suspension
gator.SetSteeringVisualizationType(vis_type)                       # primitives for steering
gator.SetWheelVisualizationType(vis_type)                          # primitives for wheels
gator.SetTireVisualizationType(vis_type)                           # primitives for tires

system = gator.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())              # truth's literal mass banner

# Add a simple primitive (box) collision shape to the chassis instead of a mesh collision.
chassis_body = gator.GetChassisBody()                              # chassis rigid body
chassis_mat = chrono.ChContactMaterialNSC()                        # NSC material (matches rigid terrain)
chassis_mat.SetFriction(0.9)                                       # chassis contact friction
chassis_mat.SetRestitution(0.01)                                   # chassis contact restitution
chassis_body.AddCollisionShape(                                    # primitive box, NOT a mesh collision
    chrono.ChCollisionShapeBox(chassis_mat, 3.0, 1.5, 0.6),
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0.3), chrono.QUNIT),
)
chassis_body.EnableCollision(True)                                 # turn on chassis collision
system.GetCollisionSystem().BindAll()                              # rebuild collision models after the edit

terrain = veh.RigidTerrain(system)                                  # rigid terrain attached to the shared system
patch_mat = chrono.ChContactMaterialNSC()                          # NSC material for the patch
patch_mat.SetFriction(0.9)                                         # terrain friction
patch_mat.SetRestitution(0.01)                                     # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # patch color
terrain.Initialize()                                               # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-specific Irrlicht window
vis.SetWindowTitle("Gator Vehicle")                                # window title
vis.SetWindowSize(1280, 1024)                                      # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)       # chase-camera track point/distance/height
vis.Initialize()                                                   # build the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # add logo after Initialize
vis.AddSkyBox()                                                    # sky box
vis.AddLightDirectional()                                         # vehicle demos use a directional light
vis.AttachVehicle(gator.GetVehicle())                            # bind the vehicle visual assets

render_step_size = 1.0 / 50.0                                      # render cadence (50 fps)
render_steps = math.ceil(render_step_size / step_size)            # physics steps between rendered frames

driver = veh.ChInteractiveDriverIRR(vis)                           # interactive keyboard driver bound to vis
# Less responsive driver: larger time-to-max means the control ramps up slowly.
steering_time = 2.0                                                # seconds 0 -> +1 steering (slow response)
throttle_time = 2.0                                                # seconds 0 -> +1 throttle (slow response)
braking_time = 1.0                                                 # seconds 0 -> +1 brake (slow response)
driver.SetSteeringDelta(render_step_size / steering_time)         # per-render steering increment
driver.SetThrottleDelta(render_step_size / throttle_time)         # per-render throttle increment
driver.SetBrakingDelta(render_step_size / braking_time)           # per-render braking increment
driver.Initialize()                                               # finalize the driver

render_every = max(1, render_steps)                               # untagged render-cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                    # spin to keep wall-clock = sim time
step_number = 0                                                  # physics step counter
while vis.Run():                                                 # real-time interactive loop
    time = system.GetChTime()                                   # current sim time

    if step_number % render_steps == 0:                         # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


    driver_inputs = driver.GetInputs()                          # current driver command

    driver.Synchronize(time)                                    # sync driver
    terrain.Synchronize(time)                                   # sync terrain
    gator.Synchronize(time, driver_inputs, terrain)            # sync vehicle with driver + terrain
    vis.Synchronize(time, driver_inputs)                      # sync visualization + HUD

    driver.Advance(step_size)                                  # advance driver
    terrain.Advance(step_size)                                 # advance terrain
    gator.Advance(step_size)                                  # advances the wrapper-owned system
    vis.Advance(step_size)                                    # advance visualization


    step_number += 1                                           # advance step counter
    realtime_timer.Spin(step_size)                            # spin in place to match wall-clock
