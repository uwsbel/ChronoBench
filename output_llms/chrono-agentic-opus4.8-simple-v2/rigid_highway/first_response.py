import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle data files

initLoc = chrono.ChVector3d(6, -70, 0.5)                             # initial vehicle location on the highway
initRot = chrono.QuatFromAngleZ(1.57)                               # face down the highway (+90 deg about Z)

vis_type = veh.VisualizationType_MESH                               # mesh visualization for all vehicle parts
chassis_collision_type = veh.CollisionType_NONE                     # no chassis collision geometry
tire_model = veh.TireModelType_TMEASY                              # prompt: TMEASY tire model

contact_method = chrono.ChContactMethod_NSC                         # NSC for rigid terrain
step_size = 1e-3                                                     # integration step
tire_step_size = step_size                                          # tire force-model step
render_step_size = 1.0 / 50.0                                       # 50 frames per second

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()                                          # full HMMWV catalog model
vehicle.SetContactMethod(contact_method)                            # NSC contact
vehicle.SetChassisCollisionType(chassis_collision_type)            # CollisionType_NONE
vehicle.SetChassisFixed(False)                                      # MANDATORY — chassis must move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))      # spawn pose
vehicle.SetTireType(tire_model)                                     # TMEASY tires
vehicle.SetTireStepSize(tire_step_size)                            # tire step
vehicle.Initialize()                                               # build the vehicle

vehicle.SetChassisVisualizationType(vis_type)                      # mesh chassis
vehicle.SetSuspensionVisualizationType(vis_type)                  # mesh suspension
vehicle.SetSteeringVisualizationType(vis_type)                    # mesh steering
vehicle.SetWheelVisualizationType(vis_type)                       # mesh wheels
vehicle.SetTireVisualizationType(vis_type)                        # mesh tires

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact

# Create the custom-mesh rigid terrain (collision + visual meshes)
patch_mat = chrono.ChContactMaterialNSC()                          # rigid-terrain contact material
patch_mat.SetFriction(0.9)                                         # road friction
patch_mat.SetRestitution(0.01)                                     # near-inelastic
terrain = veh.RigidTerrain(vehicle.GetSystem())                    # terrain attached to vehicle system
patch = terrain.AddPatch(patch_mat,                                # collision mesh patch
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj'),
    True, 0.01, False)
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(  # highway visual mesh
    veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()                # visual shape wrapper
tri_mesh_shape.SetMesh(vis_mesh)                                   # bind the visual mesh
tri_mesh_shape.SetMutable(False)                                   # static geometry
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)              # attach visual mesh to ground
terrain.Initialize()                                              # finalize terrain

# Create the vehicle Irrlicht interface
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)                     # chase-camera track point on chassis
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle-specific Irrlicht system
vis.SetWindowTitle('HMMWV on Highway')                            # window title
vis.SetWindowSize(1280, 1024)                                     # window size
vis.SetChaseCamera(trackPoint, 6.0, 0.5)                          # follow the chassis
vis.Initialize()                                                 # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # logo (after Initialize)
vis.AddLightDirectional()                                        # vehicle scenes use a directional light
vis.AddSkyBox()                                                  # sky box
vis.AttachVehicle(vehicle.GetVehicle())                          # bind vehicle visual assets

# Create the interactive driver system (steering / throttle / braking)
driver = veh.ChInteractiveDriverIRR(vis)                          # keyboard-driven interactive driver
steering_time = 1.0                                               # s to go 0 -> +1 steering
throttle_time = 1.0                                               # s to go 0 -> +1 throttle
braking_time = 0.3                                                # s to go 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)        # steering response rate
driver.SetThrottleDelta(render_step_size / throttle_time)        # throttle response rate
driver.SetBrakingDelta(render_step_size / braking_time)          # braking response rate
driver.Initialize()                                              # finalize driver

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())          # truth's literal banner

render_steps = math.ceil(render_step_size / step_size)           # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                    # real-time pacing
step_number = 0                                                  # physics step counter

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:                          # throttled rendering at 50 FPS
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                           # current driver command


    driver.Synchronize(time)                                     # update driver
    terrain.Synchronize(time)                                    # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)           # feed inputs to the vehicle
    vis.Synchronize(time, driver_inputs)                        # update vis HUD

    driver.Advance(step_size)                                    # advance driver
    terrain.Advance(step_size)                                   # advance terrain
    vehicle.Advance(step_size)                                   # advances the wrapper-owned system
    vis.Advance(step_size)                                       # advance vis

    step_number += 1                                             # next step


    realtime_timer.Spin(step_size)                              # spin to match wall-clock
