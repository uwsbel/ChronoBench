import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('./')


my_system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(my_system,  
                              100, 100, 2,  
                              1000,  
                              True,  
                              True,  
                              chrono.ChMaterialSurfaceNSC())  
ground.SetBodyFixed(True)
my_system.Add(ground)


terrain = chrono.ChBodyEasyMesh(my_system,  
                                './Highway_col.obj',  
                                './Highway_vis.obj',  
                                0.01,  
                                1000,  
                                True,  
                                True,  
                                chrono.ChMaterialSurfaceNSC())  
terrain.SetBodyFixed(True)
my_system.Add(terrain)


vehicle = chrono.ChPart_hmmwv(my_system)
vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1.5))
vehicle.SetInitRotation(chrono.Q_from_AngZ(0))
vehicle.SetContactMethod(chrono.ChMaterialSurface::NSC)
vehicle.SetTireModel(chrono.ChPart_hmmwv::TMEASY)
vehicle.SetTireStepSize(0.01)
vehicle.SetTireFriction(0.8)
vehicle.SetTireRestitution(0.4)
my_system.Add(vehicle)


driver = chrono.ChIrrNodeDriver(vehicle)
driver.SetSteeringIncrement(0.1)
driver.SetThrottleIncrement(0.1)
driver.SetBrakingIncrement(0.1)


myapplication = chronoirr.ChIrrApp(my_system, 'HMMWV Simulation', chronoirr.dimension2du(800, 600))


myapplication.SetStepManage(True)
myapplication.SetTimestep(1e-3)
myapplication.SetTryRealtime(True)


while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()