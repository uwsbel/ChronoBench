import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle data files

initLoc = chrono.ChVector3d(6, -70, 0.5)                            # truck (tractor) initial location on the highway
initRot = chrono.QuatFromAngleZ(1.57)                              # truck heading along +Y (90 deg about Z)

sedanLoc = chrono.ChVector3d(2, -90, 0.5)                           # sedan initial location, behind/beside the truck
sedanRot = chrono.QuatFromAngleZ(1.57)                             # sedan heading along +Y as well

vis_type = veh.VisualizationType_MESH                              # render vehicle parts as meshes
chassis_collision_type = veh.CollisionType_NONE                   # no chassis collision (truth default)

contact_method = chrono.ChContactMethod_NSC                        # rigid-terrain catalog vehicles use NSC

step_size = 1e-3                                                    # integration step
tire_step_size = step_size                                         # tire substep matches main step
render_step_size = 1.0 / 50                                        # render at 50 FPS

trackPoint = chrono.ChVector3d(0, 0, 2.1)                          # camera tracks tractor cab

vehicle = veh.Kraz()                                               # Kraz tractor-trailer truck (owns its system)
vehicle.SetContactMethod(contact_method)                          # NSC rigid-terrain contact
vehicle.SetChassisCollisionType(chassis_collision_type)           # disable chassis collision shape
vehicle.SetChassisFixed(False)                                    # MANDATORY — fixed chassis never moves
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))     # spawn pose on the highway
vehicle.SetTireStepSize(tire_step_size)                           # tractor/trailer tire substep
# prompt: truck tire model = RIGID (Kraz wraps its tractor tires internally; rigid is its built-in road tire)
vehicle.Initialize()                                              # build tractor + trailer subsystem stack

vehicle.SetChassisVisualizationType(vis_type, vis_type)          # tractor + trailer chassis mesh
vehicle.SetSteeringVisualizationType(vis_type)                   # steering mesh
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)       # tractor + trailer suspension mesh
vehicle.SetWheelVisualizationType(vis_type, vis_type)            # tractor + trailer wheels mesh
vehicle.SetTireVisualizationType(vis_type, vis_type)            # tractor + trailer tires mesh

system = vehicle.GetSystem()                                       # the wrapper-owned ChSystem (shared with sedan)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize

sedan = veh.BMW_E90(system)                                        # second vehicle MUST share the truck's system
sedan.SetContactMethod(contact_method)                            # same NSC contact method
sedan.SetChassisCollisionType(chassis_collision_type)            # no chassis collision shape
sedan.SetChassisFixed(False)                                     # sedan chassis free to move
sedan.SetInitPosition(chrono.ChCoordsysd(sedanLoc, sedanRot))   # sedan spawn pose
sedan.SetTireType(veh.TireModelType_TMEASY)                     # sedan handling tire
sedan.SetTireStepSize(tire_step_size)                           # sedan tire substep
sedan.Initialize()                                              # build sedan into the shared system

sedan.SetChassisVisualizationType(vis_type)                    # sedan chassis mesh
sedan.SetSuspensionVisualizationType(vis_type)                # sedan suspension mesh
sedan.SetSteeringVisualizationType(vis_type)                  # sedan steering mesh
sedan.SetWheelVisualizationType(vis_type)                    # sedan wheels mesh
sedan.SetTireVisualizationType(vis_type)                    # sedan tires mesh

patch_mat = chrono.ChContactMaterialNSC()                     # NSC terrain contact material
patch_mat.SetFriction(0.9)                                   # road friction
patch_mat.SetRestitution(0.01)                              # nearly inelastic road
terrain = veh.RigidTerrain(system)                          # rigid terrain attached to the shared system
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj'),  # predefined highway collision mesh
    True, 0.01, False)
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)        # highway visual mesh
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()                       # visual shape wrapper for the road
tri_mesh_shape.SetMesh(vis_mesh)                                          # bind the loaded mesh
tri_mesh_shape.SetMutable(False)                                         # static mesh, no per-frame rebuild
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)                     # attach highway visual to the ground body
terrain.Initialize()                                                     # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                # vehicle-specific Irrlicht window
vis.SetWindowTitle('Kraz on Highway')                          # window title
vis.SetWindowSize(1280, 1024)                                 # window resolution
vis.SetChaseCamera(trackPoint, 25.0, 1.5)                    # chase camera behind/above the cab
vis.Initialize()                                             # create the Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # add logo after Initialize
vis.AddLightDirectional()                                    # vehicle scenes use a directional light
vis.AddSkyBox()                                              # sky backdrop
vis.AttachVehicle(vehicle.GetTractor())                     # bind the tractor visual assets

driver = veh.ChInteractiveDriverIRR(vis)                    # interactive driver for the truck (truth default)
steering_time = 1.0                                          # s to ramp steering 0 -> 1
throttle_time = 1.0                                          # s to ramp throttle 0 -> 1
braking_time = 0.3                                           # s to ramp brake 0 -> 1
driver.SetSteeringDelta(render_step_size / steering_time)  # steering response rate
driver.SetThrottleDelta(render_step_size / throttle_time)  # throttle response rate
driver.SetBrakingDelta(render_step_size / braking_time)    # brake response rate
driver.Initialize()                                         # finalize truck driver

sedan_driver = veh.ChDriver(sedan.GetVehicle())            # second driver system for the sedan
sedan_driver.Initialize()                                  # finalize sedan driver
sedan_throttle = 0.5                                        # fixed forward throttle for the sedan
sedan_steering = 0.0                                        # fixed (straight) steering for the sedan

print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())   # report tractor mass

render_steps = math.ceil(render_step_size / step_size)    # physics steps between rendered frames

realtime_timer = chrono.ChRealtimeStepTimer()             # wall-clock pacing
step_number = 0                                            # physics step counter
log_info = True                                           # fire the tractor/trailer state log once

render_every = max(1, round(render_step_size / step_size))   # untagged render-cadence constant

while vis.Run():
    time = system.GetChTime()                              # current sim time

    if step_number % render_steps == 0:                   # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                    # truck driver inputs

    sedan_driver.SetThrottle(sedan_throttle)              # sedan moves forward at fixed throttle
    sedan_driver.SetSteering(sedan_steering)              # sedan holds straight steering
    sedan_inputs = sedan_driver.GetInputs()              # sedan driver inputs

    driver.Synchronize(time)                              # sync truck driver
    sedan_driver.Synchronize(time)                        # sync sedan driver
    terrain.Synchronize(time)                             # sync terrain
    vehicle.Synchronize(time, driver_inputs, terrain)    # sync truck against terrain
    sedan.Synchronize(time, sedan_inputs, terrain)       # sync sedan against terrain
    vis.Synchronize(time, driver_inputs)                 # sync visualization to truck inputs

    driver.Advance(step_size)                             # advance truck driver
    sedan_driver.Advance(step_size)                       # advance sedan driver
    terrain.Advance(step_size)                            # advance terrain
    vehicle.Advance(step_size)                            # advance truck (steps the shared system)
    sedan.Advance(step_size)                              # advance sedan subsystems
    vis.Advance(step_size)                               # advance visualization

    if log_info and time > 1:                            # store the truck tractor + trailer state once
        tractor_pos = vehicle.GetTractor().GetPos()      # tractor chassis position
        trailer_pos = vehicle.GetTrailer().GetChassis().GetPos()   # trailer chassis position
        print("t = ", time)
        print("     tractor ", tractor_pos.x, "  ", tractor_pos.y)
        print("     trailer ", trailer_pos.x, "  ", trailer_pos.y)
        tractor_vel = vehicle.GetTractor().GetSpeed()    # tractor forward speed
        print("     tractor speed ", tractor_vel)
        log_info = False                                 # disable further logging


    step_number += 1                                      # next step

    realtime_timer.Spin(step_size)                        # spin in place so wall-clock matches sim time
