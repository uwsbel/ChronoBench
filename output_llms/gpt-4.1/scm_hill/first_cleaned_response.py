import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import time


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())


system = chrono.ChSystemSMC()


initLoc = chrono.ChVectorD(0, 0, 1.0)  
initRot = chrono.Q_from_AngZ(0)        
vehicle = veh.HMMWV_Full(system)
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()


terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
terrain.SetSoilParameters(2e6,   
                          0,     
                          1.1,   
                          0,     
                          30,    
                          0.01,  
                          4e7,   
                          3e4)   


terrain.SetHeightmap("heightmap.png",  
                     100,              
                     100,              
                     0,                
                     1.0)              

terrain.Initialize(100, 100, 0.05)     


path = veh.StraightLinePath(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(50, 0, 0), 1)
target_speed = 5.0  
driver = veh.PathFollowerDriver(vehicle, path, "hmmwv/steering_controller.json", "hmmwv/speed_controller.json", target_speed)
driver.Initialize()


vis = veh.ChWheeledVehicleIrrApp(vehicle, 'HMMWV SCM Terrain Demo', irr.dimension2du(1024,768))
vis.SetSkyBox()
vis.AddTypicalLights(chrono.ChVectorD(30, 100, 120), chrono.ChVectorD(-30, -80, 100), 250, 130)
vis.AddTypicalLogo()
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()


step_size = 2e-3
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(system.GetChTime())
    terrain.Synchronize(system.GetChTime())
    vehicle.Synchronize(system.GetChTime(), driver_inputs, terrain)
    vis.Synchronize("HMMWV SCM Terrain Demo", driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    system.DoStepDynamics(step_size)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    realtime_timer.Spin(step_size)


vis.GetDevice().closeDevice()