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


vehicle = chrono.ChHMMWV(my_system,  
                         chrono.ChVectorD(0, 0, 1.5),  
                         chrono.ChQuaternionD(1, 0, 0, 0),  
                         1500,  
                         chrono.ChMaterialSurfaceNSC(),  
                         4,  
                         0.8,  
                         0.3,  
                         0.1,  
                         0.1,  
                         0.01,  
                         0.01,  
                         chrono.ChTMEasy())  
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
my_system.Add(vehicle)


driver = chrono.ChIrrNodeDriver(vehicle)
driver.SetSteeringDelta(0.01)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)


myapplication = chronoirr.ChIrrApp(my_system, 'HMMWV Simulation', chronoirr.dimension2du(800, 600))


vehicle_mesh = chrono.ChIrrNodeAsset(vehicle, './HMMWV.obj')
myapplication.AddAsset(vehicle_mesh)


myapplication.SetTimestep(1e-3)
myapplication.SetTryRealtime(True)
myapplication.SetSolverType(chrono.ChSolver.Type_APGD)
myapplication.SetSolverMaxIterations(100)


while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()