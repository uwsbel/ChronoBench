import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh

# ---
# Initialize the PyChrono environment and core components.
# ---

# Create a Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# ---
# Add the required physical systems and objects as specified.
# ---

# Create the ground body
body_floor = chrono.ChBodyEasy()
body_floor.SetBodyFixed(True)
body_floor.SetMass(1000)
body_floor.SetPos(chrono.ChVectorD(0, -0.1, 0))
body_floor.SetShapeBoxes(0.5, 0.1, 1)
system.Add(body_floor)

# Create the Viper rover
viper = veh.Viper()
viper.SetVehicleModelPath("./viper/viper.py")
viper.Initialize(system)
viper.SetPos(chrono.ChVectorD(0, 0.5, 0))
viper.SetForwardSpeed(0)

# Create the driver
driver = veh.ViperDriver()
driver.SetSteeringDelta(0.1)
driver.SetMaxSteeringAngle(0.5)  # Adjust as needed
driver.SetThrottleDelta(0.1)
driver.SetMaxThrottle(1.0)
driver.SetBrakeDelta(0.1)
driver.SetMaxBrake(1.0)
viper.SetDriver(driver)

# ---
# Set necessary default parameters such as positions, forces, and interactions.
# ---

# Set initial steering angle
initial_steering = 0.0
driver.SetSteering(initial_steering)

# ---
# Create a real-time visualization using Irrlicht with camera settings and lighting.
# ---

# Create an Irrlicht application to visualize the system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 1, -2))
vis.AddTypicalLights()

# ---
# Implement a simulation loop where the rover's steering gradually changes
# over a specified time period, updating and rendering the rover at each timestep.
# ---

# Simulation parameters
time_step = 0.01
simulation_time = 10.0
steering_change_duration = 5.0
final_steering_angle = 0.5

# Simulation loop
time = 0.0
steering_start_time = 0.0
steering_changed = False

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    # Gradually change steering angle
    if time > steering_start_time and not steering_changed:
        steering_angle = initial_steering + (time - steering_start_time) * (final_steering_angle - initial_steering) / steering_change_duration
        steering_angle = min(steering_angle, final_steering_angle) # Clamp steering angle

        driver.SetSteering(steering_angle)

        if time >= steering_start_time + steering_change_duration:
            steering_changed = True
            
    time += time_step
    if time > simulation_time:
        break