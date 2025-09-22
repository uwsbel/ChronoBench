import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.robot as viper
from pychrono import irr as chronoirr

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Set gravity in negative Y direction

# Create a ground body as a large box
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.SetName("ground")  # Name the ground body
sys.Add(ground)  # Add the ground to the simulation system

# Create the Viper rover and add it to the simulation
rover = viper.Viper(sys, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0))))

# Create the driver for the rover
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)
driver.SetSteeringDelta(0)  # Initialize steering to zero

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1280, 720)  # Set the window size for the visualization
vis.SetWindowTitle('Viper rover - Rigid terrain')  # Set the window title
vis.Initialize()  # Initialize the visualization
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # Add a camera to the visualization
vis.AddTypicalLights()  # Add typical lights for better visualization

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step  # Increment the simulation time
    steering = 0
    max_steering = math.pi / 6  # Maximum steering angle
    
    # Gradually change steering over time
    if 2 < time < 7:
        steering = max_steering * (time - 2) / 5  # Increase steering for 5 seconds
    elif 7 < time < 12:
        steering = max_steering * (12 - time) / 5  # Decrease steering for 5 seconds
    
    driver.SetSteering(steering)  # Set the steering for the rover

    # Update rover dynamics and render the scene
    rover.Update()
    vis.BeginScene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End the scene for rendering

    # Advance the simulation by one time step
    sys.DoStepDynamics(time_step)

    # Spin in place for real time to catch up
    time.sleep(time_step)