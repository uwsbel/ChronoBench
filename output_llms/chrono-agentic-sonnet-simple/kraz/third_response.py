import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial truck location and orientation
initLoc = chrono.ChVector3d(7, -60, 0.5)
initRot = chrono.QuatFromAngleZ(1.57)

# Initial sedan location and orientation
initLoc_sedan = chrono.ChVector3d(3, -65, 0.5)
initRot_sedan = chrono.QuatFromAngleZ(1.57)

vis_type = veh.VisualizationType_MESH      # mesh visualization for vehicle parts
chassis_collision_type = veh.CollisionType_NONE  # no chassis collision geometry

tire_model = veh.TireModelType_RIGID       # rigid tire model as requested

terrainHeight = 0       # flat terrain at z=0
terrainLength = 100.0   # terrain size in X
terrainWidth = 100.0    # terrain size in Y

trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)  # chase camera track point on chassis

contact_method = chrono.ChContactMethod_NSC  # NSC contact for rigid terrain

step_size = 1e-3         # simulation time step
tire_step_size = step_size  # tire sub-step size

render_step_size = 1.0 / 20  # render at 20 FPS
render_steps = math.ceil(render_step_size / step_size)  # steps per render frame

# Create the Kraz truck vehicle
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)               # chassis must move freely
truck.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
truck.Initialize()
truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)

# Create the sedan (BMW_E90) sharing the truck's ChSystem
sedan = veh.BMW_E90(truck.GetSystem())  # share system to keep both in same world
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.SetTireType(tire_model)            # rigid tire model for sedan
sedan.SetTireStepSize(tire_step_size)
sedan.Initialize()
sedan.SetChassisVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

# Set collision system on the shared ChSystem after all Initialize() calls
truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create rigid terrain using highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    veh.GetDataFile('terrain/meshes/Highway_col.obj'),  # collision mesh
    True, 0.01, False)
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)  # visual mesh
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)
tri_mesh_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)
terrain.Initialize()

# Create Irrlicht visualization attached to truck tractor
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz truck + sedan simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)  # follow truck at 25 m behind
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())  # Kraz: attach to tractor, not GetVehicle()

# Create interactive driver for truck
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0   # time to reach full steering
throttle_time = 1.0   # time to reach full throttle
braking_time = 0.3    # time to reach full braking
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Create scripted driver for sedan (fixed throttle, no steering)
driver_sedan = veh.ChDriver(sedan.GetVehicle())
driver_sedan.Initialize()

# Print vehicle mass diagnostic
print("VEHICLE MASS: ", truck.GetTractor().GetMass())

# Review-only recording setup

# State storage for truck tractor and trailer
truck_state = []

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
while vis.Run():
    time = truck.GetSystem().GetChTime()

    # Control sedan: fixed throttle forward, no steering
    driver_sedan.SetThrottle(0.8)
    driver_sedan.SetSteering(0.0)

    # Collect truck tractor and trailer state
    tractor_pos = truck.GetTractorChassisBody().GetPos()
    trailer_pos = truck.GetTrailer().GetChassis().GetBody().GetPos()
    tractor_heading = truck.GetTractorChassisBody().GetRot().GetCardanAnglesZYX().z
    trailer_heading = truck.GetTrailer().GetChassis().GetBody().GetRot().GetCardanAnglesZYX().z
    truck_state.append([time, tractor_pos, tractor_heading, trailer_pos, trailer_heading])

    # Render at specified frame rate
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()
    driver_inputs_sedan = driver_sedan.GetInputs()

    # Synchronize all modules
    driver.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs, terrain)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all modules
    driver.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)


    step_number += 1
    realtime_timer.Spin(step_size)  # spin for real-time synchronization
