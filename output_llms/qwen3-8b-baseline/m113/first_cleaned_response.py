import pychrono as chrono
from pychrono.vehicle import ChM113
from pychrono import irrlicht


system = chrono.ChSystemNSC()


terrain = chrono.ChBodyEasyBox(100, 0.1, 100, 1000, True, False)
terrain.SetPos(chrono.ChVector(0, 0, 0))
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterialSurface().SetFriction(0.8)
terrain.GetMaterialSurface().SetRestitution(0.2)
system.Add(terrain)


vehicle = ChM113()
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChVector(0, 0.5, 0))
system.Add(vehicle)


driver = chrono.ChDriver()
driver.Initialize(vehicle.GetChassisBody())


application = irrlicht.ChIrrApp(system, "M113 Simulation", irrlicht.dimension2d<int>(800, 600))
application.AddCamera(chrono.ChCamera())
application.GetCamera().SetPosition(chrono.ChVector(0, 1, -10))
application.GetCamera().LookAt(chrono.ChVector(0, 0.5, 0))
application.AddLight(chrono.ChLight(chrono.ChVector(0, 10, 10), 100))


time_step = 1.0 / 60.0
time = 0.0
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    if application.GetDevice().isKeyDown(irrlicht.KEY_UP):
        driver.SetThrottle(1.0)
    elif application.GetDevice().isKeyDown(irrlicht.KEY_DOWN):
        driver.SetThrottle(-1.0)
    else:
        driver.SetThrottle(0.0)
    
    if application.GetDevice().isKeyDown(irrlicht.KEY_LEFT):
        driver.SetSteering(1.0)
    elif application.GetDevice().isKeyDown(irrlicht.KEY_RIGHT):
        driver.SetSteering(-1.0)
    else:
        driver.SetSteering(0.0)
    
    
    driver.Synchronize(time)
    system.DoStepDynamics(time_step)
    time += time_step