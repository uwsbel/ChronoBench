import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import math

# Initialize the PyChrono environment
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Create the Chrono::Engine physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Set the collision system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the vehicle: ARTcar
vehicle = veh.ARTcar()

# Set vehicle parameters
init_pos = chrono.ChVectorD(0, 0.5, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)

# Initialize the vehicle at the specified position and orientation
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)

# Set visualization type for vehicle components
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# Initialize the vehicle
vehicle.Initialize()

# Associate a collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Add the vehicle to the system
system = vehicle.GetSystem()

# Create the terrain
terrain = veh.RigidTerrain(system)

# Define terrain parameters
terrain_length = 100.0
terrain_width = 100.0
terrain_height = 0.0

# Create a patch with specified dimensions
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, 
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrain_height), 
                                         chrono.QUNIT),
                        terrain_length, terrain_width)

# Set terrain texture
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Initialize the terrain
terrain.Initialize()

# Create the interactive driver system
driver = veh.ChInteractiveDriverIRR()

# Set the time response for steering and throttle inputs
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1

driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)

# Create the Irrlicht visualization application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('ARTcar Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, -8))
vis.AddTypicalLights()

# Initialize the driver system (this must be done after the visualization system is initialized)
driver.Initialize()

# Set up simulation parameters
step_size = 1e-3
render_step_size = 1.0/50  # 50 FPS
render_steps = int(render_step_size / step_size)

# Simulation loop
time = 0
step_number = 0
render_frame = 0

print("Starting simulation...")
print("Use WASD keys to control the vehicle:")
print("W - Throttle")
print("S - Brake") 
print("A - Steer Left")
print("D - Steer Right")

while vis.Run():
    time = system.GetChTime()
    
    # Render scene every render_steps
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Synchronize the vehicle systems
    # (synchronize the vehicle subsystems with the current driver inputs)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    # Advance the dynamics of the vehicle
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    
    # Advance the simulation by one step
    system.DoStepDynamics(step_size)
    
    step_number += 1
    
    # Optional: Print some vehicle information every second
    if step_number % 1000 == 0:
        vehicle_pos = vehicle.GetPos()
        vehicle_speed = vehicle.GetSpeed()
        print(f"Time: {time:.2f}s, Position: ({vehicle_pos.x:.2f}, {vehicle_pos.y:.2f}, {vehicle_pos.z:.2f}), Speed: {vehicle_speed:.2f} m/s")

print("Simulation completed.")