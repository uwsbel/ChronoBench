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
vehicle.SetInitRotation(chrono.Q_from_AngX(chrono.CH_C_PI / 2))
vehicle.SetContactMethod(chrono.ChMaterialSurface::NSC)
vehicle.SetTireModel(chrono.ChPart_hmmwv::TMEASY)
vehicle.SetTireStepSize(0.01)
vehicle.SetTireFriction(0.8)
vehicle.SetTireElasticModulus(200e6)
vehicle.SetTireDamping(10e3)
vehicle.SetTireCohesion(100)
vehicle.SetTireFrictionScale(1.0)
vehicle.SetTireRadius(0.35)
vehicle.SetTireWidth(0.15)
vehicle.SetTireMass(50)
vehicle.SetSuspensionStiffness(100e3)
vehicle.SetSuspensionDamping(10e3)
vehicle.SetSuspensionLength(0.2)
vehicle.SetSteeringSpeed(2 * chrono.CH_C_PI)
vehicle.SetThrottleSpeed(10)
vehicle.SetBrakingSpeed(10)
my_system.Add(vehicle)


driver = chrono.ChIrrNodeDriver(vehicle)
driver.SetSteeringDelta(chrono.CH_C_PI / 100)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)


myapplication = chronoirr.ChIrrApp(my_system, 'HMMWV Simulation', chronoirr.dimension2du(800, 600))


myapplication.AddTypicalSky()
myapplication.AddTypicalCamera(chronoirr.vector3df(0, 0, 2))
myapplication.AddTypicalLights()
myapplication.AddMesh(terrain, chronoirr.vector3df(0, 0, 0), chronoirr.vector3df(1, 1, 1))
myapplication.AddMesh(vehicle.GetChassisBody(), chronoirr.vector3df(0, 0, 0), chronoirr.vector3df(1, 1, 1))


while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStepDynamics(1e-3)
    myapplication.EndScene()
    my_system.Update(chrono.ChTime(1e-3))
    driver.Update()
    myapplication.SetTimestep(1e-3)
    myapplication.SetTryRealtime(True)
    myapplication.GetDevice().setWindowCaption('HMMWV Simulation - FPS: ' + str(myapplication.GetDevice().getVideoDriver().getFPS()))