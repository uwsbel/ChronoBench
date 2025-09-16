import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('./chrono_data/')


my_system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(my_system,  
                              100, 100, 2,  
                              1000,  
                              True,  
                              True,  
                              chrono.ChMaterialSurfaceNSC())  
ground.SetBodyFixed(True)
my_system.Add(ground)


my_truck = veh.ChPart(ground)
my_truck.SetMass(10000)
my_truck.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
my_truck.SetPos(chrono.ChVectorD(0, 0, 1))
my_system.Add(my_truck)


tire = veh.ChTMEasy(my_system)
tire.SetLongitudinalStiffness(100000)
tire.SetLateralStiffness(100000)
tire.SetCamberStiffness(100000)
tire.SetFrictionCoefficient(0.8)
my_truck.AddTire(tire, chrono.ChVectorD(0, 0, -1))


driver = veh.ChRealtimeDriver(my_system)
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)
my_truck.AddDriver(driver)


myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono Truck Simulation',
                                   chronoirr.dimension2du(800, 600))


myapplication.AddTypicalCamera(chronoirr.vector3df(0, 0, 1))
myapplication.AddTypicalLights()
myapplication.AddSkyBox()
myapplication.SetSymbolscale(0.02)
myapplication.SetShowInfos(True)


myapplication.SetTerrainTexture(chrono.GetChronoDataFile('textures/pavement.jpg'))
myapplication.SetLogoPicture(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


myapplication.AssetBindAll()
myapplication.AssetUpdateAll()
myapplication.Simulate(0, 100)


while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()