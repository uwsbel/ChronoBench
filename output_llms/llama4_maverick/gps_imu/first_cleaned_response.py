import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np


out_dir = "HMMWV_SIMULATION"





veh.InitLog(chrono.GetLogLevel_ALL)
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(-5, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()




terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)), chrono.ChVectorD(100, 100, 0))
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterial(terrain_material)
terrain.Initialize()




driver = veh.ChDriver(vehicle, True)
driver.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Simulation')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AttachVehicle(vehicle)





imu = veh.ChIMUSensor(vehicle.GetChassisBody(),  
                      100,  
                      chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),  
                      1,  
                      0,  
                      0,  
                      0)  
vehicle.GetSystem().AddSensor(imu)


gps = veh.ChGPSSensor(vehicle.GetChassisBody(),  
                      10,  
                      chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),  
                      0,  
                      0,  
                      0)  
vehicle.GetSystem().AddSensor(gps)




while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)

    
    vehicle.Advance(0.01)
    terrain.Advance(0.01)
    vis.Advance(0.01)

    
    vehicle.GetSystem().Update()

    
    print(f"Vehicle mass: {vehicle.GetVehicle().GetMass()}")

    
    vis.Render()