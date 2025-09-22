import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(chrono.ChCollisionType_MESH)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
hmmwv.Initialize(sys)


terrain = veh.RigidTerrain(hmmwv.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


driver = veh.ChIrrGuiDriver()
hmmwv.AddDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")
vis.SetCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 0, 0))
vis.SetTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, -3))
vis.AddLightWithShadow(chrono.ChVectorD(2, 2, -2), chrono.ChColor(1.2, 1.2, 1.2), 1)
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 1, -3))
vis.AddLightWithShadow(chrono.ChVectorD(2, 2, -2), chrono.ChColor(1.2, 1.2, 1.2), 1)
vis.AttachSystem(sys)


imu_sensor = veh.ChSensorIMU(hmmwv.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
gps_sensor = veh.ChSensorGPS(hmmwv.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.QUNIT)


hmmwv.AddSensor(imu_sensor)
hmmwv.AddSensor(gps_sensor)


time = 0
while time < 10:
    
    sys.DoStepDynamics(0.01)
    time += 0.01

    
    imu_sensor.ProcessFrame()
    gps_sensor.ProcessFrame()

    
    print("Vehicle mass: ", hmmwv.GetChassisBody().GetMass())

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    driver.Synchronize(time)


vis.GetDevice().run()
vis.GetDevice().closeDevice()