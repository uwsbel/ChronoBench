import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

print(veh)

"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# --------------
# Create systems
# --------------

# Create the first vehicle (BMW E90)
vehicle1 = veh.BMW_E90()
vehicle1.SetContactMethod(contact_method)
vehicle1.SetChassisCollisionType(chassis_collision_type)
vehicle1.SetChassisFixed(False)
vehicle1.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle1.SetTireType(tire_model)
vehicle1.SetTireStepSize(tire_step_size)
vehicle1.Initialize()

vehicle1.SetChassisVisualizationType(vis_type)
vehicle1.SetSuspensionVisualizationType(vis_type)
vehicle1.SetSteeringVisualizationType(vis_type)
vehicle1.SetWheelVisualizationType(vis_type)
vehicle1.SetTireVisualizationType(vis_type)

vehicle1.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain for vehicle 1
patch_mat1 = chrono.ChContactMaterialNSC()
patch_mat1.SetFriction(0.9)
patch_mat1.SetRestitution(0.01)
terrain1 = veh.RigidTerrain(vehicle1.GetSystem())
patch1 = terrain1.AddPatch(patch_mat1, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch1.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain1.Initialize()

# Create the second vehicle (e.g., Sedan)
vehicle2 = veh.BMW_E90()
vehicle2.SetContactMethod(contact_method)
vehicle2.SetChassisCollisionType(chassis_collision_type)
vehicle2.SetChassisFixed(False)
# Set initial position offset in Y to avoid overlap
initLoc2 = chrono.ChVector3d(0, 5, 0.5)
vehicle2.SetInitPosition(chrono.ChCoordsysd(initLoc2, initRot))
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(tire_step_size)
vehicle2.Initialize()

vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)

vehicle2.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain for vehicle 2 with different texture
patch_mat2 = chrono.ChContactMaterialNSC()
patch_mat2.SetFriction(0.9)
patch_mat2.SetRestitution(0.01)
terrain2 = veh.RigidTerrain(vehicle2.GetSystem())
patch2 = terrain2.AddPatch(patch_mat2, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain2.Initialize()

# Create the vehicle Irrlicht interface for vehicle 1
vis1 = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis1.SetWindowTitle('Sedan 1')
vis1.SetWindowSize(1280, 1024)
vis1.SetChaseCamera(trackPoint, 6.0, 0.5)
vis1.Initialize()
vis1.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis1.AddLightDirectional()
vis1.AddSkyBox()
vis1.AttachVehicle(vehicle1.GetVehicle())

# Create the vehicle Irrlicht interface for vehicle 2
vis2 = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis2.SetWindowTitle('Sedan 2')
vis2.SetWindowSize(1280, 1024)
# Offset camera for second vehicle
trackPoint2 = chrono.ChVector3d(0, 5, 1.8)
vis2.SetChaseCamera(trackPoint2, 6.0, 0.5)
vis2.Initialize()
vis2.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis2.AddLightDirectional()
vis2.AddSkyBox()
vis2.AttachVehicle(vehicle2.GetVehicle())

# Create drivers for both vehicles
driver1 = veh.ChInteractiveDriverIRR(vis1)
driver2 = veh.ChInteractiveDriverIRR(vis2)

# Set the time response for steering and throttle keyboard inputs for driver 1
steering_time = 1.0  # seconds
throttle_time = 1.0
braking_time = 0.3

driver1.SetSteeringDelta(render_step_size / steering_time)
driver1.SetThrottleDelta(render_step_size / throttle_time)
driver1.SetBrakingDelta(render_step_size / braking_time)
driver1.Initialize()

# Set the time response for driver 2
driver2.SetSteeringDelta(render_step_size / steering_time)
driver2.SetThrottleDelta(render_step_size / throttle_time)
driver2.SetBrakingDelta(render_step_size / braking_time)
driver2.Initialize()

# Output vehicle masses
print("VEHICLE 1 MASS: ", vehicle1.GetVehicle().GetMass())
print("VEHICLE 2 MASS: ", vehicle2.GetVehicle().GetMass())

# Number of simulation steps between rendering
render_steps = math.ceil(render_step_size / step_size)

# Initialize real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# ---------------
# Simulation loop
# ---------------
while vis1.Run() and vis2.Run():
    time = vehicle1.GetSystem().GetChTime()  # Both vehicles share the same system time

    # Render scene and output POV-Ray data periodically
    if (step_number % render_steps == 0):
        vis1.BeginScene()
        vis1.Render()
        vis1.EndScene()
        # For second viewer, synchronize rendering
        vis2.BeginScene()
        vis2.Render()
        vis2.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_input1 = driver1.GetInputs()
    driver_input2 = driver2.GetInputs()

    # Apply sinusoidal steering input to both drivers
    steering_input1 = math.sin(2 * math.pi * 0.1 * time)  # 0.1 Hz oscillation
    steering_input2 = math.sin(2 * math.pi * 0.1 * time + math.pi/4)  # phase shifted

    # Override steering inputs with sinusoidal values
    driver_input1.m_steering = steering_input1
    driver_input2.m_steering = steering_input2

    # Synchronize modules
    driver1.Synchronize(time)
    terrain1.Synchronize(time)
    vehicle1.Synchronize(time, driver_input1, terrain1)

    driver2.Synchronize(time)
    terrain2.Synchronize(time)
    vehicle2.Synchronize(time, driver_input2, terrain2)

    # Advance modules
    driver1.Advance(step_size)
    terrain1.Advance(step_size)
    vehicle1.Advance(step_size)

    driver2.Advance(step_size)
    terrain2.Advance(step_size)
    vehicle2.Advance(step_size)

    # Synchronize visualization
    vis1.Synchronize(time, driver_input1)
    vis2.Synchronize(time, driver_input2)

    # Advance visualization
    vis1.Advance(step_size)
    vis2.Advance(step_size)

    # Increment frame counter
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)