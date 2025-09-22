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


my_truck = veh.ChPart('MAN_10t')
my_truck.SetMass(10000)
my_truck.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
my_system.Add(my_truck)


my_tire = veh.ChTMEasy()
my_tire.SetMass(50)
my_tire.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
my_tire.SetLongitudinalStiffness(100000)
my_tire.SetLateralStiffness(50000)
my_system.Add(my_tire)


my_truck.AddAsset(my_tire)


my_driver = veh.ChDriver()
my_driver.Initialize()
my_system.Add(my_driver)


my_truck.AddAsset(my_driver)


my_truck.SetPos(chrono.ChVectorD(0, 0, 1.5))
my_truck.SetForce(chrono.ChVectorD(0, 0, -9810))
my_tire.SetForce(chrono.ChVectorD(0, 0, -490.5))


application = chronoirr.ChIrrApp(my_system, 'PyChrono Simulation', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(0, 0, 3), chronoirr.vector3df(0, 0, 0))


application.SetChaseCamera(my_truck)


application.AddTexture('terrain', './chrono_data/textures/terrain.jpg')


application.AddLogo('./chrono_data/textures/logo.png')


application.AssetBindAll()
application.AssetUpdateAll()
application.Simulate(0.01)


print('Simulation started. Press Ctrl+C to exit.')
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()