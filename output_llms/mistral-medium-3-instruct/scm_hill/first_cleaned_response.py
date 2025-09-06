import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np
import os






step_size = 1e-3


t_end = 10.0


trackPoint = chrono.ChVectorD(0.0, 0.0, 1.0)






vehicle = veh.ChVehicle()
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.SetInitFwdVel(5)  


terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngZ(0)))
terrain.Initialize(10, 10, 0.5)  


terrain.SetSoilParameters(1e6,    
                         0.01,   
                         1.1,    
                         0,      
                         30,     
                         0.01)   


nx, ny = 50, 50
height_map = np.zeros((nx, ny))
for i in range(nx):
    for j in range(ny):
        x = (i - nx/2) * 0.2
        y = (j - ny/2) * 0.2
        height_map[i,j] = 0.1 * np.exp(-(x*x + y*y)/10)
terrain.SetHeightMap(height_map)


driver = veh.ChDriver()
driver.SetSteeringFunction(veh.ChFunction_Const(0))  
driver.SetThrottleFunction(veh.ChFunction_Const(0.5))  






hmmwv = veh.HMMWV()
hmmwv.Initialize(vehicle.GetSystem(), veh.GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json"))


hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))






vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on Deformable Terrain")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 3), trackPoint)
vis.AddTypicalLights()
vis.SetCameraMovementType(chronoirr.ChCameraMovementType.FOLLOW)
vis.SetCameraFollowPoint(trackPoint)
vis.SetCameraFollowDistance(8)
vis.SetCameraFollowHeight(2)






real_time = True
time_step = step_size
current_time = 0


while current_time < t_end:
    
    vis.BeginScene()

    
    vis.Render()

    
    vis.SetSymbolScale(0.1)
    vis.ShowCOGmarkers(True)
    vis.ShowFrames(True)

    
    driver.Synchronize(current_time)
    driver.Advance(step_size)

    
    vehicle.Synchronize(current_time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    vehicle.Advance(step_size)

    
    terrain.Synchronize(current_time)
    terrain.Advance(step_size)

    
    vis.EndScene()

    
    current_time += step_size

    
    if real_time:
        chrono.ChTime.sleep(chrono.ChTime.GetTime() - current_time)


vis.Close()