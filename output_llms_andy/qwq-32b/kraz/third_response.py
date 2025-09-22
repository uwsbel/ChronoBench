import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
initLoc_truck = chrono.ChVectorD(0, 0, 1)
initRot_truck = chrono.ChQuaternionD(1, 0, 0, 0)

initLoc_sedan = chrono.ChVectorD(5, 0, 1)
initRot_sedan = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization and collision types
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

# Tire model for truck (changed to RIGID)
tire_model_truck = veh.TireModelType_RIGID

# Terrain parameters (changed to highway texture)
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Simulation settings
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
render_step_size = 1.0 / 50

# Create Kraz truck
vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc_truck, initRot_truck))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)
vehicle.SetTireType(tire_model_truck)  # Set tire model to RIGID

# Create sedan
sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(veh.CollisionType_PRIMITIVES)
sedan.SetInitPosition(chrono.ChCoordsysD(initLoc_sedan, initRot_sedan))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

# Create terrain with highway texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 2.1), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())
vis.AttachVehicle(sedan)  # Attach sedan to visualization

# Drivers
driver_truck = veh.ChInteractiveDriverIRR(vis)
driver_truck.SetSteeringDelta(render_step_size / 1.0)
driver_truck.SetThrottleDelta(render_step_size / 1.0)
driver_truck.SetBrakingDelta(render_step_size / 0.3)
driver_truck.Initialize()

driver_sedan = veh.ChDriver()
driver_sedan.Initialize(sedan.GetVehicle(), 0.0)  # Initialize with vehicle and time step

# File for storing truck states
state_file = open('truck_states.csv', 'w')
state_file.write("time, tractor_x, tractor_y, tractor_z, trailer_x, trailer_y, trailer_z\n")

# Simulation loop variables
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render
    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Truck driver inputs
    driver_inputs_truck = driver_truck.GetInputs()
    
    # Sedan driver fixed inputs
    driver_sedan.SetThrottle(0.5)
    driver_sedan.SetSteering(0.0)
    driver_inputs_sedan = driver_sedan.GetInputs()

    # Synchronize and advance modules
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs_truck, terrain)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs_truck)

    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    # Store truck states
    tractor = vehicle.GetTractor()
    trailer = vehicle.GetTrailer()
    tractor_pos = tractor.GetPos()
    trailer_pos = trailer.GetPos() if trailer else chrono.ChVectorD(0,0,0)
    state_file.write(f"{time}, {tractor_pos.x}, {tractor_pos.y}, {tractor_pos.z}, "
                    f"{trailer_pos.x}, {trailer_pos.y}, {trailer_pos.z}\n")

    step_number += 1
    realtime_timer.Spin(step_size)

# Cleanup
state_file.close()

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
initLoc_truck = chrono.ChVectorD(0, 0, 1)
initRot_truck = chrono.ChQuaternionD(1, 0, 0, 0)

initLoc_sedan = chrono.ChVectorD(5, 0, 1)
initRot_sedan = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization and collision types
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

# Tire model for truck (changed to RIGID)
tire_model_truck = veh.TireModelType_RIGID

# Terrain parameters (changed to highway texture)
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Simulation settings
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
render_step_size = 1.0 / 50

# Create Kraz truck
vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc_truck, initRot_truck))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)
vehicle.SetTireType(tire_model_truck)  # Set tire model to RIGID

# Create sedan
sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(veh.CollisionType_PRIMITIVES)
sedan.SetInitPosition(chrono.ChCoordsysD(initLoc_sedan, initRot_sedan))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

# Create terrain with highway texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 2.1), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())
vis.AttachVehicle(sedan)  # Attach sedan to visualization

# Drivers
driver_truck = veh.ChInteractiveDriverIRR(vis)
driver_truck.SetSteeringDelta(render_step_size / 1.0)
driver_truck.SetThrottleDelta(render_step_size / 1.0)
driver_truck.SetBrakingDelta(render_step_size / 0.3)
driver_truck.Initialize()

driver_sedan = veh.ChDriver()
driver_sedan.Initialize(sedan.GetVehicle(), 0.0)  # Initialize with vehicle and time step

# File for storing truck states
state_file = open('truck_states.csv', 'w')
state_file.write("time, tractor_x, tractor_y, tractor_z, trailer_x, trailer_y, trailer_z\n")

# Simulation loop variables
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render
    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Truck driver inputs
    driver_inputs_truck = driver_truck.GetInputs()
    
    # Sedan driver fixed inputs
    driver_sedan.SetThrottle(0.5)
    driver_sedan.SetSteering(0.0)
    driver_inputs_sedan = driver_sedan.GetInputs()

    # Synchronize and advance modules
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs_truck, terrain)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs_truck)

    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    # Store truck states
    tractor = vehicle.GetTractor()
    trailer = vehicle.GetTrailer()
    tractor_pos = tractor.GetPos()
    trailer_pos = trailer.GetPos() if trailer else chrono.ChVectorD(0,0,0)
    state_file.write(f"{time}, {tractor_pos.x}, {tractor_pos.y}, {tractor_pos.z}, "
                    f"{trailer_pos.x}, {trailer_pos.y}, {trailer_pos.z}\n")

    step_number += 1
    realtime_timer.Spin(step_size)

# Cleanup
state_file.close()