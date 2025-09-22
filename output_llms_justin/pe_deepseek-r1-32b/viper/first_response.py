import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

# Set the Chrono data path
chrono.SetChronoDataPath(chrono.GetChronoDataDir())

# Create the Chrono system
system = chrono.ChSystemSMC()
system.SetGravity(chrono.ChVectorD(0, -9.81, 0))

# Create the ground body
ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)

# Add visual and collision shapes to ground
ground_shape = chrono.ChVisualShapeBox(chrono.ChVectorD(20, 1, 20))
ground.AddVisualShape(ground_shape)
ground_coll = chrono.ChCollisionShapeBox(chrono.ChVectorD(20, 1, 20))
ground.AddCollisionShape(ground_coll)

system.Add(ground)

# Create the Viper rover
rover = veh.ViperRover()
rover.Initialize(system, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0)))

# Create the driver
driver = veh.ViperDriver()
rover.SetDriver(driver)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Viper Rover Simulation")
vis.AttachSystem(system)
vis.SetCameraPosition(chrono.ChVectorD(5, 5, 5))
vis.SetCameraRotation(chrono.ChVectorD(-45, 45, 0))
vis.AddTypicalLights()

# Add a skybox for better visualization
vis.AddSkyBox()

# Simulation parameters
time_step = 0.01
simulation_time = 0.0
max_time = 10.0

# Simulation loop
while simulation_time < max_time:
    # Update driver inputs
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.5  # Constant throttle
    
    # Gradually change steering angle
    steering_angle = math.sin(simulation_time) * 0.5  # Varies between -0.5 and 0.5 radians
    driver_inputs.m_steering = steering_angle
    
    driver.SetInputs(driver_inputs)
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Check for user input to close
    key = vis.GetKeyboard()
    if key == ord(' '):
        break
    
    simulation_time += time_step

# Close the visualization
vis.Close()