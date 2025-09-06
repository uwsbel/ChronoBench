import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 1e-3

# Simulation end time
t_end = 10

# Time interval for changing steering
steering_time = 5

# =============================================================================
# Initialize the Chrono system
# =============================================================================

# Create the Chrono system
system = chrono.ChSystemNSC()

# Set gravity (negative Z direction)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# =============================================================================
# Create the terrain
# =============================================================================

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.1))
ground.SetBodyFixed(True)
ground.GetVisualModel().SetTexture(veh.GetDataFile("textures/concrete.jpg"))
system.Add(ground)

# =============================================================================
# Create the Viper rover
# =============================================================================

# Create the Viper rover system
viper = veh.Viper()
viper.SetContactMethod(chrono.ChContactMethod_NSC)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoordinator(0, 0, 0.5))
viper.SetInitFwdVel(0.5)

# Initialize the rover
viper.Initialize()

# Add the rover to the system
viper.GetSystem().AddToSystem(&system)

# =============================================================================
# Create the driver system
# =============================================================================

# Create a simple driver
driver = veh.ChDriver()

# Set the driver to control the Viper
driver.SetSteeringDelta(0.5)
driver.SetThrottleDelta(0.2)
driver.SetBrakingDelta(0.2)

# =============================================================================
# Create the visualization system
# =============================================================================

# Create the Irrlicht application
app = chronoirr.ChIrrApp(system, "Viper Rover Simulation", chrono.ChVectorD(1280, 720))

# Customize some settings
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(0, 5, 2), chrono.ChVectorD(0, 0, 0.5))
app.SetTimestep(step_size)
app.AssetBindAll()
app.AssetUpdateAll()

# =============================================================================
# Simulation loop
# =============================================================================

# Simulation time
time = 0

while app.GetDevice().run() and time < t_end:
    # Update the time
    time = system.GetChTime()

    # Calculate steering input that changes over time
    steering = np.sin(2 * np.pi * time / steering_time)

    # Set driver inputs
    driver.SetSteering(steering)
    driver.SetThrottle(0.5)

    # Update the driver
    driver.Synchronize(time)
    driver.Advance(step_size)

    # Update the rover
    viper.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())

    # Perform the simulation step
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()

    # Print some information
    if int(time) % 1 == 0:
        print(f"Time: {time:.2f}s, Steering: {steering:.2f}, Position: {viper.GetChassisBody().GetPos().x:.2f}, {viper.GetChassisBody().GetPos().y:.2f}")