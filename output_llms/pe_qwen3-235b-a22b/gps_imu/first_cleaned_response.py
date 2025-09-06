import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', ''))


sys = chrono.ChSystemNSC()


vehicle = veh.HMMWV()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_ALL)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize(sys)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 200, 100)
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()


driver = veh.ChDataDriver(vehicle)
driver.Initialize()


imu = sens.ChIMUSensor(
    vehicle.GetChassisBody(),
    100,  
    chrono.ChVector3d(0, 0, 0),  
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))  
)
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetNoise(0.01)
sys.AddSensor(imu)

gps = sens.ChGPSSensor(
    vehicle.GetChassisBody(),
    10,  
    chrono.ChVector3d(0, 0, 0),  
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))  
)
gps.SetName("GPS Sensor")
gps.SetReferenceLocation(chrono.ChVector3d(0, 0, 0))
sys.AddSensor(gps)


manager = sens.ChSensorManager(sys)
manager.AddSensor(imu)
manager.AddSensor(gps)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation with Sensors')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


time_step = 1e-3
while vis.Run():
    time = sys.GetChTime()
    
    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    
    
    manager.Update()
    
    
    sys.DoStepDynamics(time_step)
    
    
    print(f"Time: {time:.3f} s, Vehicle mass: {vehicle.GetTotalMass():.2f} kg")
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()