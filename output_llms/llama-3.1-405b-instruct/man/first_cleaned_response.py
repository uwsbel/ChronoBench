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
my_truck.SetPos(chrono.ChVectorD(0, 0, 1.5))


tire = veh.ChTMEasyTire()
tire.SetLongitudinalStiffness(100000)
tire.SetLateralStiffness(100000)
tire.SetCamberStiffness(10000)
my_truck.AddTire(tire, chrono.ChVectorD(0.5, 0, -0.5), chrono.ChQuaternionD(1, 0, 0, 0))


driver = veh.ChIrrNodeDriver(my_system)
driver.Initialize()
driver.SetSteeringIncrement(0.01)
driver.SetThrottleIncrement(0.01)
driver.SetBrakingIncrement(0.01)


myapplication = chronoirr.ChIrrApp(my_system, 'MAN 10t Truck Simulation', chronoirr.dimension2du(800, 600))


mycamera = chronoirr.ChIrrNodeCamera(chronoirr.core.vector3df(0, 0, 0), chronoirr.core.vector3df(0, 0, 0))
myapplication.AddCamera(mycamera)
myapplication.SetChaseCamera(mycamera, my_truck)


mylight = chronoirr.ChIrrNodeLight(chronoirr.core.vector3df(0, 20, 0), chronoirr.core.vector3df(0, 0, 0), 1)
myapplication.AddLight(mylight)


myapplication.AddSkyBox()


myapplication.AddTexture(ground, chronoirr.core.stringc('terrain.jpg'))
myapplication.AddLogo(chronoirr.core.stringc('logo.png'))


myapplication.AssetBindAll()
myapplication.AssetUpdateAll()
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetSolverMaxIterations(100)
myapplication.SetTimestep(0.01)

while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()