import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial truck location and orientation
truck_initLoc = chrono.ChVectorD(0, 0, 1.0)  # Changed location
truck_initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Sedan initial location and orientation
sedan_initLoc = chrono.ChVectorD(10, 0, 0.5)  # Added
sedan_initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization types
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Tire model for truck (changed to RIGID)
truck_tire_model = veh.TireModelType_RIGID
# Sedan tire model can remain as default (TMEASY?) unless specified, but instructions don't mention changing it.

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera track point for truck
truck_trackPoint = chrono.ChVectorD(0, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

render_step_size = 1.0 / 50

# Create the truck vehicle
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysD(truck_initLoc, truck_initRot))
truck.SetTireType(truck_tire_model)  # Set tire model to RIGID
truck.Initialize()

# Visualization for truck
truck.SetChassisVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type)
truck.SetTireVisualizationType(vis_type)

# Create the sedan vehicle
sedan = veh.Jeep()  # Assuming Jeep is a sedan-like model
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetInitPosition(chrono.ChCoordsysD(sedan_initLoc, sedan_initRot))
sedan.Initialize()

# Visualization for sedan
sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

# Create the terrain
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())  # Using truck's system
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)  # Changed to highway texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(truck_trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck)
vis.AttachVehicle(sedan)  # Added sedan to visualization

# Drivers
truck_driver = veh.ChInteractiveDriverIRR(vis)  # Interactive driver for truck
truck_driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)
truck_driver.Initialize()
truck.SetDriver(truck_driver)  # Link driver to truck

# Sedan driver with fixed inputs
sedan_driver = veh.ChDriver()
sedan_driver.SetThrottle(0.5)  # Fixed throttle
sedan_driver.SetSteering(0)    # No steering
sedan.SetDriver(sedan_driver)

# Initialize simulation variables
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Storage for truck's tractor and trailer states
tractor_states = []
trailer_states = []

# Simulation loop
while vis.Run():
    time = truck.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

        # Store states
        tractor = truck.GetTractor()
        trailer = truck.GetTrailer()
        tractor_states.append((tractor.GetPos(), tractor.GetRot()))
        trailer_states.append((trailer.GetPos(), trailer.GetRot()))

    # Update truck driver inputs
    truck_inputs = truck_driver.GetInputs()
    truck.Synchronize(time, truck_inputs, terrain)
    truck_driver.Synchronize(time)
    vis.Synchronize(time, truck_inputs)

    # Sedan's driver inputs are fixed, so no need for GetInputs
    sedan.Synchronize(time, sedan_driver.GetInputs(), terrain)

    # Advance all modules
    truck_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Modified initial truck location and orientation
truck_initLoc = chrono.ChVectorD(0, 0, 1.0)
truck_initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Added initial sedan location and orientation
sedan_initLoc = chrono.ChVectorD(10, 0, 0.5)
sedan_initRot = chrono.ChQuaternionD(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

# Truck tire model changed to RIGID
truck_tire_model = veh.TireModelType_RIGID

terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
render_step_size = 1.0 / 50
render_steps = math.ceil(render_step_size / step_size)

# Create truck
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysD(truck_initLoc, truck_initRot))
truck.SetTireType(truck_tire_model)  # Set to RIGID
truck.Initialize()

# Visualization for truck
truck.SetChassisVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type)
truck.SetTireVisualizationType(vis_type)

# Create sedan
sedan = veh.Jeep()  # Assuming Jeep is a sedan-like vehicle
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetInitPosition(chrono.ChCoordsysD(sedan_initLoc, sedan_initRot))
sedan.Initialize()

# Visualization for sedan
sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

# Create terrain with highway texture
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Truck and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(truck_initLoc + chrono.ChVectorD(0, 0, 2.1), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck)
vis.AttachVehicle(sedan)

# Truck driver (interactive)
truck_driver = veh.ChInteractiveDriverIRR(vis)
truck_driver.SetSteeringDelta(render_step_size / 1.0)
truck_driver.SetThrottleDelta(render_step_size / 1.0)
truck_driver.SetBrakingDelta(render_step_size / 0.3)
truck_driver.Initialize()
truck.SetDriver(truck_driver)

# Sedan driver (fixed inputs)
sedan_driver = veh.ChDriver()
sedan_driver.SetThrottle(0.5)  # Fixed throttle
sedan_driver.SetSteering(0)    # Straight ahead
sedan.SetDriver(sedan_driver)

# Storage for truck states
tractor_states = []
trailer_states = []

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = truck.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Store tractor and trailer states
        tractor = truck.GetTractor()
        trailer = truck.GetTrailer()
        tractor_states.append((tractor.GetPos(), tractor.GetRot()))
        trailer_states.append((trailer.GetPos(), trailer.GetRot()))

    # Truck updates
    truck_inputs = truck_driver.GetInputs()
    truck.Synchronize(time, truck_inputs, terrain)
    truck_driver.Synchronize(time)
    vis.Synchronize(time, truck_inputs)

    # Sedan updates (fixed inputs)
    sedan.Synchronize(time, sedan_driver.GetInputs(), terrain)

    # Advance all modules
    truck_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)