import pychrono.core as chrono
import pychrono.vehicle as veh

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity in Y direction

# Create the terrain (ground body)
ground = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.25, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Create the Viper rover
# Assuming Viper is a predefined vehicle model in pychrono.vehicle
# If not, a similar vehicle can be created using the VehicleFactory
viper = veh.Viper(system)

# Initialize the rover
viper.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
viper.SetChassisPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
viper.SetChassisFixed(False)

# Create the driver
driver = veh.ChIrrGuiDriver()
driver.Initialize()
viper.GetVehicle().SetDriver(driver)

# Set up Irrlicht visualization
application = veh.ChIrrApp(viper.GetVehicle(), 'Viper Rover Simulation', chrono.dimension(800, 600))
application.GetSceneManager().AddCamera(chrono.ChCameraSceneNode(
    application.GetSceneManager(), 
    application.GetVideoDriver(), 
    chrono.ChVectorD(0, 2, -4),  # Camera position
    chrono.ChVectorD(0, 0, 0),   # Look at point
    30                          # FOV
))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation parameters
time_step = 0.01
total_time = 10.0
steering_rate = 0.1  # radians per second
max_steering_angle = 0.5  # radians
current_steering = 0.0
steering_direction = 1  # 1 for increasing, -1 for decreasing

# Simulation loop
t = 0
while t < total_time:
    # Gradually change steering angle over time
    current_steering += steering_direction * steering_rate * time_step
    if current_steering > max_steering_angle:
        current_steering = max_steering_angle
        steering_direction = -1
    elif current_steering < -max_steering_angle:
        current_steering = -max_steering_angle
        steering_direction = 1
    
    # Apply steering to the vehicle
    viper.GetVehicle().SetSteeringAngle(current_steering)

    # Begin scene rendering
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    # Advance the simulation
    system.DoStepDynamics(time_step)
    application.GetVideoDriver().run()
    application.GetVideoDriver().waitFrame()

    t += time_step