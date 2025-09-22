import chrono as ch
import chrono.irrlicht as chirr
import math

# Initialize PyChrono environment and core components
ch.ChEngine.Initialize()

# Create the Chrono system with gravity
system = ch.ChSystemNSC()
system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))

# Create the ground body
ground = ch.ChBodyEasyTerrain(
    100, 100,  20, 20, 
    ch.ChVector3d(0, -0.1, 0), 
    ch.ChQuaterniond(1, 0, 0, 0),
    False, False
)
ground.SetPos(ch.ChVector3d(0, -0.2, 0))
system.AddBody(ground)

# Load the Viper rover
viper = ch.ChVehicle()
viper.Initialize(ch.ChVector3d(0, 0.5, 0), system)

# Create and set the driver for the Viper rover
driver = ch.ChVehicleDriver(viper)
driver.SetSteeringDelta(0.0)  # initial steering

# Create the Irrlicht application for visualization
app = chirr.ChIrrApp(system, "Viper Rover Simulation")
app.SetCameraPosition(ch.ChVector3d(0, 5, -10))
app.SetCameraTarget(ch.ChVector3d(0, 0, 0))

# Enable lighting and add default camera settings
app.EnableDefaultLights()
app.SetVerbose(False)

# Simulation loop parameters
timestep = 0.05
end_time = 10.0  # seconds
steering_change_time = 5.0  # seconds
initial_steering = 0.0
final_steering = 30.0  # degrees

# Simulation loop
t = 0
while t < end_time:
    # Update steering over time
    steering = initial_steering + (final_steering - initial_steering) * (t / steering_change_time) if t < steering_change_time else final_steering
    driver.SetSteeringDelta(math.radians(steering))
    
    # Update the system
    system.Update(timestep)
    
    # Render the scene
    app.Render()
    app.SpinEventLoop(0.02)
    
    # Increment time
    t += timestep

# Clean up
del app
ch.ChEngine.Finalize()