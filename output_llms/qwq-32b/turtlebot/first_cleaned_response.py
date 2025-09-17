import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetTimestep(0.01)


ground = chrono.ChBodyEasyBox(20, 0.1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)


turtlebot = vehicle.Turtlebot()
turtlebot.Initialize(my_system, True, "Pirelli")  
turtlebot.GetChassis().SetPos(chrono.ChVectorD(0, 0.1, 0))
turtlebot.GetChassis().SetRot(chrono.Q_from_Euler_123(0, 0, 0))
turtlebot.SetupVisualization()


vis = irr.ChIrrApp(my_system, 'Turtlebot Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalLights()
vis.SetCameraPosition(chrono.ChVectorD(2, 2, 2))
vis.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
vis.AddAsset(turtlebot.GetVisualizationAssets())
vis.AddAsset(ground.GetAssetContainer())

vis.SetTimestep(my_system.GetSettings().timestep())
vis.SetRealtimeFactor(1)


vis.Initialize()
vis.AddCamera()

while vis.Run():
    current_time = my_system.GetChTime()
    
    
    if current_time < 2:
        
        turtlebot.GetDriveline().SetLeftDriveVelocity(5)
        turtlebot.GetDriveline().SetRightDriveVelocity(5)
    elif 2 <= current_time <5:
        
        turtlebot.GetDriveline().SetLeftDriveVelocity(0)
        turtlebot.GetDriveline().SetRightDriveVelocity(5)
    elif 5 <= current_time <8:
        
        turtlebot.GetDriveline().SetLeftDriveVelocity(5)
        turtlebot.GetDriveline().SetRightDriveVelocity(0)
    else:
        
        turtlebot.GetDriveline().SetLeftDriveVelocity(0)
        turtlebot.GetDriveline().SetRightDriveVelocity(0)
    
    
    turtlebot.Synchronize(my_system)
    turtlebot.Update(0, 0)  
    vis.DoStep()

import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetTimestep(0.01)


ground = chrono.ChBodyEasyBox(20, 0.1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)


turtlebot = vehicle.Turtlebot()
turtlebot.Initialize(my_system, True, "Pirelli")  
turtlebot.GetChassis().SetPos(chrono.ChVectorD(0, 0.1, 0))  
turtlebot.SetupVisualization()


vis = irr.ChIrrApp(my_system, 'Turtlebot Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalLights()
vis.SetCameraPosition(chrono.ChVectorD(2, 2, 2))
vis.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
vis.AddAsset(turtlebot.GetVisualizationAssets())
vis.AddAsset(ground.GetAssetContainer())

vis.SetTimestep(my_system.GetSettings().timestep())
vis.SetRealtimeFactor(1)


vis.Initialize()
vis.AddCamera()

while vis.Run():
    current_time = my_system.GetChTime()
    
    
    if current_time < 2:
        
        turtlebot.GetDriveline().SetLeftDriveVelocity(5)
        turtlebot.GetDriveline().SetRightDriveVelocity(5)
    elif 2 <= current_time < 5:
        
        turtlebot.GetDriveline().SetLeftDriveVelocity(0)
        turtlebot.GetDriveline().SetRightDriveVelocity(5)
    elif 5 <= current_time < 8:
        
        turtlebot.GetDriveline().SetLeftDriveVelocity(5)
        turtlebot.GetDriveline().SetRightDriveVelocity(0)
    else:
        
        turtlebot.GetDriveline().SetLeftDriveVelocity(0)
        turtlebot.GetDriveline().SetRightDriveVelocity(0)
    
    
    turtlebot.Synchronize(my_system)
    turtlebot.Update(0, 0)  
    vis.DoStep()