import pychrono as chrono
import pychrono.robot as viper
import pychrono.irrlicht as chronoirr
import math

# ---------------------------------------------------------------------
# Create the simulation system and set up basic parameters
# ---------------------------------------------------------------------

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  # Set gravity in negative Z direction

# Set solver parameters for better stability
system.SetSolverMaxIterations(50)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# ---------------------------------------------------------------------
# Create the terrain (ground body)
# ---------------------------------------------------------------------

# Create ground body
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))
ground.SetBodyFixed(True)
ground.SetName("ground")

# Set ground material properties
ground_material = chrono.ChMaterialSurfaceNSC()
ground_material.SetFriction(0.8)
ground_material.SetRestitution(0.1)
ground.SetMaterialSurface(ground_material)

# Add ground to system
system.Add(ground)

# ---------------------------------------------------------------------
# Create and initialize the Viper rover
# ---------------------------------------------------------------------

# Create Viper rover
rover = viper.ChViper(system)

# Set rover initial position and orientation
init_pos = chrono.ChVectorD(0, 0, 0.2)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))

# Create driver for the rover
driver = viper.ChViperDriver()

# Set initial driver inputs
driver.SetSteering(0.0)      # Initial steering angle
driver.SetDriving(0.3)       # Forward drive (30% throttle)

# ---------------------------------------------------------------------
# Create the Irrlicht visualization
# ---------------------------------------------------------------------

# Create the Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()

# Add lighting
vis.AddLight(chrono.ChVectorD(10, 10, 10), 200, chrono.ChColor(1, 1, 1))
vis.AddLight(chrono.ChVectorD(-10, 10, 10), 200, chrono.ChColor(0.7, 0.7, 0.8))

# Set visualization options
vis.EnableShadows()

# ---------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------

# Time step
time_step = 1e-3
time_end = 20.0

# Steering parameters
max_steering_angle = 0.6  # Maximum steering angle (radians)
steering_period = 8.0     # Time period for one complete steering cycle

# Rendering frame rate
render_step = 1.0/50  # 50 FPS
render_frame = 0

# Initialize simulation time
time = 0

print("Starting Viper rover simulation...")
print(f"Simulation will run for {time_end} seconds")
print("Watch as the rover's steering gradually changes over time!")

# ---------------------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------------------

while vis.Run() and time < time_end:
    
    # Calculate time-varying steering input
    # Use sinusoidal function to gradually change steering over time
    steering_angle = max_steering_angle * math.sin(2 * math.pi * time / steering_period)
    
    # Update driver inputs
    driver.SetSteering(steering_angle)
    driver.SetDriving(0.3)  # Maintain constant forward speed
    
    # Synchronize the driver with the rover
    driver.Synchronize(time)
    rover.Update()
    
    # Advance the simulation
    system.DoStepDynamics(time_step)
    
    # Update visualization every render_step
    if time >= render_frame * render_step:
        # Update camera to follow rover
        rover_pos = rover.GetChassis().GetPos()
        camera_pos = rover_pos + chrono.ChVectorD(3, 3, 1.5)
        target_pos = rover_pos + chrono.ChVectorD(0, 0, 0.5)
        
        vis.GetActiveCamera().setPosition(chronoirr.vector3df(
            camera_pos.x, camera_pos.y, camera_pos.z))
        vis.GetActiveCamera().setTarget(chronoirr.vector3df(
            target_pos.x, target_pos.y, target_pos.z))
        
        # Render the scene
        vis.BeginScene()
        vis.Render()
        
        # Display simulation info
        vis.GetGUIEnvironment().getBuiltInFont().draw(
            f"Time: {time:.2f}s",
            chronoirr.recti(10, 10, 200, 30),
            chronoirr.SColor(255, 255, 255, 255))
        
        vis.GetGUIEnvironment().getBuiltInFont().draw(
            f"Steering: {steering_angle:.3f} rad",
            chronoirr.recti(10, 35, 250, 55),
            chronoirr.SColor(255, 255, 255, 255))
        
        vis.GetGUIEnvironment().getBuiltInFont().draw(
            f"Rover Pos: ({rover_pos.x:.2f}, {rover_pos.y:.2f}, {rover_pos.z:.2f})",
            chronoirr.recti(10, 60, 350, 80),
            chronoirr.SColor(255, 255, 255, 255))
        
        vis.EndScene()
        render_frame += 1
    
    # Increment time
    time += time_step
    
    # Print progress every few seconds
    if int(time) % 5 == 0 and time > 0:
        progress = (time / time_end) * 100
        if progress <= 100:
            print(f"Simulation progress: {progress:.1f}%")

print("Simulation completed!")
print("Closing visualization...")

# Clean up
vis.GetDevice().closeDevice()