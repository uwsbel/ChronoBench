import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths correctly
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Truck parameters
truck_initLoc = chrono.ChVectorD(0, 0, 1)  # Changed initial position
truck_initRot = chrono.ChQuaternionD(chrono.Q_from_AngAxis(chrono.CH_C_PI / 6, chrono.ChVectorD(0,1,0)))  # 30° Y-axis rotation

# Sedan parameters
sedan_initLoc = chrono.ChVectorD(5, 0, 0.5)  # Initial position ahead of truck
sedan_initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization types
vis_type = veh.VisualizationType_MESH

# Collision types
truck_chassis_collision_type = veh.CollisionType_NONE
sedan_chassis_collision_type = veh.CollisionType_PRIMITIVES

# Tire models
truck_tire_model = veh.TireModelType_RIGID  # Changed to rigid
sedan_tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Simulation settings
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
render_step_size = 1.0 / 50

# Create truck
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(truck_chassis_collision_type)
truck.SetInitPosition(chrono.ChCoordsysD(truck_initLoc, truck_initRot))

# Set tire model to RIGID for all axles
for axle in truck.GetAxles():
    axle.GetTire().SetType(truck_tire_model)

truck.Initialize()
truck.SetChassisVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type)
truck.SetTireVisualizationType(vis_type)

# Create sedan
sedan = veh.Jeep()  # Example sedan vehicle
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(sedan_chassis_collision_type)
sedan.SetInitPosition(chrono.ChCoordsysD(sedan_initLoc, sedan_initRot))
sedan.SetTireType(sedan_tire_model)  # Set tire model
sedan.Initialize()
sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

# Terrain setup with highway texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())  # Use truck's system
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Truck and Sedan Simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(truck.GetTractor().COM express(), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(sedan)  # Attach sedan to visualization

# Drivers
truck_driver = veh.ChInteractiveDriverIRR(vis)
truck_driver.SetSteeringDelta(render_step_size / 1.0)
truck_driver.SetThrottleDelta(render_step_size / 1.0)
truck_driver.SetBrakingDelta(render_step_size / 0.3)
truck_driver.Initialize()

# Sedan driver with fixed controls
sedan_driver = veh.ChDriver()
sedan_driver.SetThrottle(0.5)  # Fixed throttle
sedan_driver.SetSteering(0)    # Straight ahead

# State storage
truck_states = []
sedan_states = []

# Simulation loop
step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = truck.GetSystem().GetChTime()
    
    # Render
    if step_number % (render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Truck driver inputs
    truck_inputs = truck_driver.GetInputs()
    
    # Sedan driver inputs (fixed)
    sedan_inputs = sedan_driver.GetInputs()
    
    # Update modules
    truck_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_inputs, terrain)
    sedan.Synchronize(time, sedan_inputs, terrain)
    vis.Synchronize(time, truck_inputs)
    
    # Advance simulation
    truck_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)
    
    # Store states
    truck_states.append(truck.GetTractor().GetPos())
    if hasattr(truck, 'GetTrailer'):
        trailer_pos = truck.GetTrailer().GetPos()
        truck_states[-1] = (truck_states[-1], trailer_pos)
    sedan_states.append(sedan.GetChassis().GetPos())
    
    step_number += 1
    realtime_timer.Spin(step_size)

# Output stored states
print("Truck tractor positions:", truck_states)
print("Sedan positions:", sedan_states)