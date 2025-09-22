import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os # For creating data directory if it doesn't exist

# Ensure Chrono data paths are correctly set
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# --- Simulation parameters ---

# Initial truck location and orientation (changed)
initLoc_truck = chrono.ChVector3d(-10, -2, 1.0) # Changed from (0,0,0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0) # Kept orientation

# Initial sedan location and orientation (added)
initLoc_sedan = chrono.ChVector3d(0, 2, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
# Truck chassis collision (can be PRIMITIVES, MESH, or NONE)
truck_chassis_collision_type = veh.CollisionType_PRIMITIVES # Changed from NONE for better interaction
# Sedan chassis collision
sedan_chassis_collision_type = veh.CollisionType_PRIMITIVES

# Type of tire model
tire_model_truck = veh.TireModelType_RIGID  # Changed for truck
tire_model_sedan = veh.TireModelType_TMEASY # For sedan

# Terrain type: Using a predefined highway mesh
# Variables for old terrain type (terrainLength, terrainWidth) are no longer needed.
terrain_initial_height = 0.0 # Reference height for the mesh if needed for placement

# Point on chassis tracked by the camera (relative to truck chassis)
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75) # Adjusted for Kraz chassis

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False # Not used in this script, but good for debugging

# Simulation step sizes
step_size = 2e-3 # Adjusted step size for stability with two vehicles
tire_step_size = step_size # Tire step size, often same as simulation step

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Output directory
out_dir = "KRAZ_SEDAN_SIMULATION"
if not os.path.exists(out_dir):
    os.makedirs(out_dir)

# --- Create the Kraz truck ---
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(truck_chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.SetTireType(tire_model_truck) # Set tire type for truck
truck.Initialize()

# Set visualization types for the truck
truck.SetChassisVisualizationType(vis_type) # Corrected: single argument
truck.SetSteeringVisualizationType(vis_type) # Correct: single argument for steering links
truck.SetSuspensionVisualizationType(vis_type, vis_type) # Correct: front/rear
truck.SetWheelVisualizationType(vis_type, vis_type) # Correct: front/rear
truck.SetTireVisualizationType(vis_type) # Corrected: single argument

# --- Create the Sedan ---
sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(sedan_chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.SetTireType(tire_model_sedan) # Set tire type for sedan
sedan.Initialize()

# Set visualization types for the sedan
sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type)

# Get the system instance (both vehicles belong to the same system after initialization)
system = truck.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# --- Create the terrain ---
# Use ChMaterialSurfaceNSC (or SMC if contact_method was SMC)
patch_mat = chrono.ChMaterialSurfaceNSC() # Corrected: ChMaterialSurfaceNSC
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
# Add highway mesh patch
# Ensure "terrain/meshes/long_road.obj" exists in the Chrono data directory
# The mesh is typically placed at (0,0,0) unless an offset is provided in ChCoordsysd
highway_mesh_file = veh.GetDataFile("terrain/meshes/long_road.obj")
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0,0,terrain_initial_height), chrono.QUNIT), highway_mesh_file, "highway", 0.01) # sweep radius

# Optional: Set visual properties for the mesh patch if not defined in OBJ/MTL
# patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) # May not apply well to arbitrary OBJ
patch.GetGroundBody().GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.5)) # Example: set color of the mesh

terrain.Initialize()


# --- Create the Irrlicht visualization system ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan on Highway Demo')
vis.SetWindowSize(1280, 1024)
# Camera will follow the truck
vis.SetChaseCamera(trackPoint, 12.0, 0.5) # chase distance, height of camera
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetVehicle()) # Attach Kraz truck
vis.AttachVehicle(sedan.GetVehicle()) # Attach Sedan


# --- Create the driver systems ---
# Driver for Kraz (interactive)
driver_truck = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)
driver_truck.Initialize()

# Driver for Sedan (scripted)
driver_sedan = veh.ChDriver(sedan.GetVehicle())


# --- Simulation run ---
# Output vehicle masses
print("TRUCK MASS: ", truck.GetMass())
print("SEDAN MASS: ", sedan.GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter and timers
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

# Data storage for truck state
truck_time_history = []
truck_pos_history = []
truck_rot_history = []
truck_chassis_body = truck.GetChassisBody() # Get a reference to the truck's chassis body

# Main simulation loop
while vis.Run():
    time = system.GetChTime()

    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Store truck state at render frames
        current_pos = truck_chassis_body.GetPos()
        current_rot = truck_chassis_body.GetRot()
        truck_time_history.append(time)
        truck_pos_history.append([current_pos.x, current_pos.y, current_pos.z])
        truck_rot_history.append([current_rot.e0, current_rot.e1, current_rot.e2, current_rot.e3])

    # Get driver inputs for Kraz
    driver_inputs_truck = driver_truck.GetInputs()

    # Set driver inputs for Sedan (fixed throttle, no steering)
    driver_sedan.SetThrottle(0.5) # Move forward with 50% throttle
    driver_sedan.SetSteering(0.0) # No steering
    driver_sedan.SetBraking(0.0)   # No braking
    driver_inputs_sedan = driver_sedan.GetInputs()


    # Update modules (process inputs from other modules)
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time) # Synchronize sedan driver (updates its internal state from SetThrottle/Steering)
    
    terrain.Synchronize(time)
    
    truck.Synchronize(time, driver_inputs_truck, terrain)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    
    vis.Synchronize(time, driver_inputs_truck) # Irrlicht sync typically takes primary driver inputs

    # Advance simulation for one timestep for all modules
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    
    terrain.Advance(step_size)
    
    truck.Advance(step_size)
    sedan.Advance(step_size)
    
    system.DoStepDynamics(step_size) # This is done by vehicle.Advance() internally

    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# End of simulation
# Example: Save truck data to a file (optional)
# You can use numpy or pandas for more robust data saving if needed
with open(os.path.join(out_dir, "truck_state.csv"), "w") as f:
    f.write("Time,PosX,PosY,PosZ,RotE0,RotE1,RotE2,RotE3\n")
    for i in range(len(truck_time_history)):
        pos = truck_pos_history[i]
        rot = truck_rot_history[i]
        f.write(f"{truck_time_history[i]},{pos[0]},{pos[1]},{pos[2]},{rot[0]},{rot[1]},{rot[2]},{rot[3]}\n")

print(f"Truck state data saved to {os.path.join(out_dir, 'truck_state.csv')}")