import pychrono as chrono
import pychrono.viper as viper
import pychrono.irrlicht as irrlicht

# Initialize the Chrono system with gravity
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a rigid ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(100, 1, 100)  # Large flat ground
ground.SetPos(chrono.ChVectorD(0, -1, 0))  # Position the ground
system.Add(ground)

# Initialize the Viper rover
rover = viper.ChViper(system)
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))  # Start position and orientation

# Set up the driver for the Viper rover
driver = viper.ChViperDriver()
rover.SetDriver(driver)

# Configure Irrlicht visualization
application = irrlicht.ChIrrApp(system, 'Viper Rover Simulation', irrlicht.dimension2du(1280, 720))
application.AddTypicalLogo()
application.AddTypicalCamera(irrlicht.vector3df(0, 3, -6), irrlicht.vector3df(0, 0, 0))  # Camera setup
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation parameters
end_time = 10.0  # Duration for steering change (seconds)
max_steering = 1.0  # Maximum steering input (-1 to 1)

# Simulation loop
while application.GetDevice().run():
    current_time = system.GetChTime()
    
    # Gradually change steering from -max to +max over end_time
    if current_time <= end_time:
        steering = -max_steering + (2 * max_steering) * (current_time / end_time)
    else:
        steering = max_steering  # Hold at maximum steering after end_time
    
    # Update driver inputs
    driver.SetSteering(steering)
    driver.SetThrottle(0.3)  # Apply constant throttle for forward motion

    # Advance the simulation
    system.DoStepDynamics(1e-3)  # Fixed time step of 0.001 seconds

    # Update and render the scene
    application.BeginScene()
    application.DrawAll()
    application.EndScene()