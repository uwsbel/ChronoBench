import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(100, 1, 100)  # x, y, z half-dimensions
ground.SetPos(chrono.ChVectorD(0, -1, 0))  # Position the ground
system.Add(ground)

# Create the Viper rover
rover = veh.Viper(system)
rover.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
rover.Initialize()

# Create the driver
driver = veh.ViperSimpleDriver(rover)
driver.Initialize()

# Set up Irrlicht visualization
application = irr.ChIrrApp(system, 'Viper Rover Simulation', irr.dimension2du(800, 600))
application.AddTypicalLogo()
application.AddTypicalCamera(chrono.ChVectorD(0, 2, -5), rover.GetChassis().GetPos())
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop parameters
time_step = 1e-3  # 0.001 seconds
end_time = 20.0  # seconds

# Simulation loop
while application.GetDevice().run():
    current_time = system.GetChTime()
    if current_time > end_time:
        break

    # Update steering: ramp up to 0.5 over 10s, then ramp down to 0 over next 10s
    if current_time < 10.0:
        steering = 0.5 * (current_time / 10.0)
    else:
        steering = 0.5 * (1.0 - (current_time - 10.0) / 10.0)
    steering = max(0.0, min(0.5, steering))  # Clamp steering between 0 and 0.5
    driver.SetSteering(steering)

    # Update driver and rover
    driver.Synchronize(current_time)
    rover.Synchronize(current_time, driver.GetInputs(), time_step)

    # Advance simulation
    system.DoStepDynamics(time_step)

    # Render
    application.BeginScene()
    application.DrawAll()
    application.EndScene()