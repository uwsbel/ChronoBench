import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr

# Create the Chrono system
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetTimestep(0.01)

# Create ground plane
ground = chrono.ChBodyEasyBox(20, 0.1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)

# Initialize Turtlebot
turtlebot = vehicle.Turtlebot()
turtlebot.Initialize(my_system, True, "Pirelli")  # Use suspension and default tire
turtlebot.GetChassis().SetPos(chrono.ChVectorD(0, 0.1, 0))
turtlebot.GetChassis().SetRot(chrono.Q_from_Euler_123(0, 0, 0))
turtlebot.SetupVisualization()

# Visualization setup
vis = irr.ChIrrApp(my_system, 'Turtlebot Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalLights()
vis.SetCameraPosition(chrono.ChVectorD(2, 2, 2))
vis.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
vis.AddAsset(turtlebot.GetVisualizationAssets())
vis.AddAsset(ground.GetAssetContainer())

vis.SetTimestep(my_system.GetSettings().timestep())
vis.SetRealtimeFactor(1)

# Simulation loop
vis.Initialize()
vis.AddCamera()

while vis.Run():
    current_time = my_system.GetChTime()
    
    # Control logic based on time
    if current_time < 2:
        # Move straight
        turtlebot.GetDriveline().SetLeftDriveVelocity(5)
        turtlebot.GetDriveline().SetRightDriveVelocity(5)
    elif 2 <= current_time <5:
        # Turn left (right wheel faster)
        turtlebot.GetDriveline().SetLeftDriveVelocity(0)
        turtlebot.GetDriveline().SetRightDriveVelocity(5)
    elif 5 <= current_time <8:
        # Turn right (left wheel faster)
        turtlebot.GetDriveline().SetLeftDriveVelocity(5)
        turtlebot.GetDriveline().SetRightDriveVelocity(0)
    else:
        # Stop
        turtlebot.GetDriveline().SetLeftDriveVelocity(0)
        turtlebot.GetDriveline().SetRightDriveVelocity(0)
    
    # Update vehicle and step simulation
    turtlebot.Synchronize(my_system)
    turtlebot.Update(0, 0)  # Steering and throttle set to 0, driveline controlled directly
    vis.DoStep()

import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr

# Initialize Chrono system with gravity
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetTimestep(0.01)

# Create rigid ground plane
ground = chrono.ChBodyEasyBox(20, 0.1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)

# Initialize Turtlebot robot
turtlebot = vehicle.Turtlebot()
turtlebot.Initialize(my_system, True, "Pirelli")  # Enable suspension, default tire
turtlebot.GetChassis().SetPos(chrono.ChVectorD(0, 0.1, 0))  # Position above ground
turtlebot.SetupVisualization()

# Set up visualization with Irrlicht
vis = irr.ChIrrApp(my_system, 'Turtlebot Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalLights()
vis.SetCameraPosition(chrono.ChVectorD(2, 2, 2))
vis.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
vis.AddAsset(turtlebot.GetVisualizationAssets())
vis.AddAsset(ground.GetAssetContainer())

vis.SetTimestep(my_system.GetSettings().timestep())
vis.SetRealtimeFactor(1)

# Simulation loop
vis.Initialize()
vis.AddCamera()

while vis.Run():
    current_time = my_system.GetChTime()
    
    # Motor control based on time intervals
    if current_time < 2:
        # Move straight
        turtlebot.GetDriveline().SetLeftDriveVelocity(5)
        turtlebot.GetDriveline().SetRightDriveVelocity(5)
    elif 2 <= current_time < 5:
        # Turn left (right wheel faster)
        turtlebot.GetDriveline().SetLeftDriveVelocity(0)
        turtlebot.GetDriveline().SetRightDriveVelocity(5)
    elif 5 <= current_time < 8:
        # Turn right (left wheel faster)
        turtlebot.GetDriveline().SetLeftDriveVelocity(5)
        turtlebot.GetDriveline().SetRightDriveVelocity(0)
    else:
        # Stop
        turtlebot.GetDriveline().SetLeftDriveVelocity(0)
        turtlebot.GetDriveline().SetRightDriveVelocity(0)
    
    # Update vehicle and step simulation
    turtlebot.Synchronize(my_system)
    turtlebot.Update(0, 0)  # Steering=0, throttle=0 (controlled via driveline)
    vis.DoStep()