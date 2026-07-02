import pychrono as chrono
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


terrain = chrono.ChBodyEasyBox(100, 0.1, 100, 1000, True, False)
terrain.SetFixed(True)
terrain.SetPos(chrono.ChVector3d(0, -0.5, 0))
system.AddBody(terrain)


vehicle = chrono.ChVehicleHMMWV()
vehicle.SetChassisFixed(False)
vehicle.SetChassisBody(chrono.ChBodyEasyBox(2.5, 1.5, 0.8, 1000, True, False))
vehicle.SetChassisBodyPos(chrono.ChVector3d(0, 0.75, 0))
vehicle.SetChassisBodyRot(chrono.QuatFromAngleX(0.1))
system.Add(vehicle.GetVehicleBody())


imu = chrono.ChBody()
imu.SetName("IMU Sensor")
imu.SetFixed(True)
imu.SetPos(vehicle.GetChassisBody().GetPos())
imu.SetRot(vehicle.GetChassisBody().GetRot())
system.AddBody(imu)


gps = chrono.ChBody()
gps.SetName("GPS Sensor")
gps.SetFixed(True)
gps.SetPos(vehicle.GetChassisBody().GetPos())
gps.SetRot(vehicle.GetChassisBody().GetRot())
system.AddBody(gps)


driver = chrono.ChDriverHMMWV()
driver.SetThrottle(0.5)
driver.SetSteering(0.0)
driver.SetBraking(0.0)
vehicle.SetDriver(driver)


visual = chrono.ChVisualSystemIrrlicht()
visual.SetWindowSize(1024, 768)
visual.SetWindowTitle("HMMWV Simulation")
visual.Initialize()
visual.AddSkyBox()
visual.AddCamera(chrono.ChVector3d(0, 2, 5), chrono.ChVector3d(0, 0, 0))


step = 0
while step < 1000:
    
    imu.SetPos(vehicle.GetChassisBody().GetPos())
    imu.SetRot(vehicle.GetChassisBody().GetRot())
    gps.SetPos(vehicle.GetChassisBody().GetPos())
    gps.SetRot(vehicle.GetChassisBody().GetRot())
    
    
    vehicle.Advance(step * 0.01)
    driver.Advance(step * 0.01)
    
    
    visual.BeginScene()
    visual.RenderSkyBox()
    visual.RenderCamera()
    visual.RenderVehicle(vehicle)
    visual.RenderTerrain(terrain)
    visual.RenderBody(imu, chrono.ChColor(1, 0, 0))
    visual.RenderBody(gps, chrono.ChColor(0, 1, 0))
    visual.EndScene()
    
    
    print(f"Vehicle mass: {vehicle.GetVehicleBody().GetMass()} kg")
    
    step += 1

visual.Close()