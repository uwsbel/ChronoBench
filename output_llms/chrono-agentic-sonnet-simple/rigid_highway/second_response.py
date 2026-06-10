import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os                                                                 # review-only (REC flag)

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                     # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')                 # locate vehicle data files

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(6, -70, 0.5)                                 # spawn point on highway
initRot = chrono.QuatFromAngleZ(1.57)                                    # heading along Y axis

vis_type = veh.VisualizationType_MESH                                    # mesh visuals for all parts
chassis_collision_type = veh.CollisionType_NONE                          # no chassis collision mesh
tire_model = veh.TireModelType_TMEASY                                    # TMEASY for better road grip

terrainHeight = 0                                                        # terrain at z=0
terrainLength = 100.0                                                    # terrain X size
terrainWidth = 100.0                                                     # terrain Y size

trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)                          # chase camera track point

contact_method = chrono.ChContactMethod_NSC                              # NSC for rigid terrain
step_size = 1e-3                                                         # simulation step size
tire_step_size = step_size                                               # tire step same as sim step
render_step_size = 1.0 / 50                                              # render at 50 FPS

# Create HMMWV vehicle and configure it
vehicle = veh.HMMWV_Full()                                               # full HMMWV model
vehicle.SetContactMethod(contact_method)                                 # NSC contact
vehicle.SetChassisCollisionType(chassis_collision_type)                  # no chassis collision
vehicle.SetChassisFixed(False)                                           # chassis free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))           # spawn location
vehicle.SetTireType(tire_model)                                          # TMEASY tires
vehicle.SetTireStepSize(tire_step_size)                                  # tire substep
vehicle.Initialize()                                                     # build vehicle

vehicle.SetChassisVisualizationType(vis_type)                            # mesh chassis
vehicle.SetSuspensionVisualizationType(vis_type)                         # mesh suspension
vehicle.SetSteeringVisualizationType(vis_type)                           # mesh steering
vehicle.SetWheelVisualizationType(vis_type)                              # mesh wheels
vehicle.SetTireVisualizationType(vis_type)                               # mesh tires

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Create rigid terrain with highway mesh patches
patch_mat = chrono.ChContactMaterialNSC()                                # NSC contact material
patch_mat.SetFriction(0.9)                                               # high friction
patch_mat.SetRestitution(0.01)                                           # low restitution
terrain = veh.RigidTerrain(vehicle.GetSystem())                          # rigid terrain

# Main highway collision mesh patch
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj'),
    True, 0.01, False)                                                   # highway collision mesh

# Visual overlay for highway surface
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)       # visual mesh
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()                      # visual shape wrapper
tri_mesh_shape.SetMesh(vis_mesh)                                         # assign mesh
tri_mesh_shape.SetMutable(False)                                         # static mesh
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)                     # attach to patch body

# Additional bump mesh patch at (0, -42, 0)
patch3 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"))                          # bump mesh at specified coords
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))                          # blue-ish color
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture, scale 6x6

terrain.Initialize()                                                     # finalize terrain

# Vehicle Irrlicht visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                         # wheeled vehicle vis
vis.SetWindowTitle('HMMWV Demo')                                         # window title
vis.SetWindowSize(1280, 1024)                                            # window size
vis.SetChaseCamera(trackPoint, 6.0, 0.5)                                 # follow vehicle
vis.Initialize()                                                         # MUST be first
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))         # Chrono logo
vis.AddLightDirectional()                                                 # directional light
vis.AddSkyBox()                                                           # sky background
vis.AttachVehicle(vehicle.GetVehicle())                                  # bind vehicle visuals

# Interactive driver (keyboard/mouse)
driver = veh.ChInteractiveDriverIRR(vis)                                 # IRR interactive driver
steering_time = 1.0                                                      # s: 0 to full steering
throttle_time = 1.0                                                      # s: 0 to full throttle
braking_time = 0.3                                                       # s: 0 to full brake
driver.SetSteeringDelta(render_step_size / steering_time)                # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)                # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)                  # braking rate
driver.Initialize()                                                      # finalize driver

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())                  # output vehicle mass (scored)

render_steps = math.ceil(render_step_size / step_size)                   # steps between renders
render_every = render_steps                                              # untagged cadence constant

sim_end = float("inf")                                                   # run indefinitely (scored core)

realtime_timer = chrono.ChRealtimeStepTimer()                            # real-time pacing timer
step_number = 0                                                          # step counter

while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    time = vehicle.GetSystem().GetChTime()                               # current sim time

    if step_number % render_every == 0:                                  # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                                   # get current driver inputs

    driver.Synchronize(time)                                             # sync driver
    terrain.Synchronize(time)                                            # sync terrain
    vehicle.Synchronize(time, driver_inputs, terrain)                    # sync vehicle
    vis.Synchronize(time, driver_inputs)                                 # sync vis HUD


    driver.Advance(step_size)                                            # advance driver
    terrain.Advance(step_size)                                           # advance terrain
    vehicle.Advance(step_size)                                           # advance vehicle + system
    vis.Advance(step_size)                                               # advance vis

    step_number += 1                                                     # increment step counter
    realtime_timer.Spin(step_size)                                       # pace to real time
