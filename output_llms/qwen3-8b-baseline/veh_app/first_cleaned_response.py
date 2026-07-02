import pychrono as chrono
from pychrono import irrlicht
from pychrono.irrlicht import ChIrrApp
from pychrono.vehicle import ChVehicle, ChDriver, ChContactMaterial, ChMaterialSurface


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))


terrain = chrono.ChBodyEasyBox(chrono.ChVectorD(0, 0, 0), 100, 1, 100, 1000, True, False)
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetFixed(True)
system.Add(terrain)



chassis = chrono.ChBodyEasyBox(chrono.ChVectorD(0, 0.5, 0), 2, 1, 0.5, 1000, True, False)
chassis.SetPos(chrono.ChVectorD(0, 0.5, 0))
chassis.SetMaterialSurface(ChMaterialSurface.FRICTION_0_3)
chassis.SetVisualizationType(chrono.ChVisualizationType_MESH)
system.Add(chassis)


wheel_radius = 0.3
wheel_length = 0.6
wheel_mass = 50


wheel_fl = chrono.ChBodyEasyCylinder(chrono.ChVectorD(-1, 0.25, 0), wheel_radius, wheel_length, wheel_mass, True, False)
wheel_fl.SetPos(chrono.ChVectorD(-1, 0.25, 0))
wheel_fl.SetMaterialSurface(ChMaterialSurface.FRICTION_0_8)
wheel_fl.SetVisualizationType(chrono.ChVisualizationType_NONE)
system.Add(wheel_fl)


wheel_fr = chrono.ChBodyEasyCylinder(chrono.ChVectorD(1, 0.25, 0), wheel_radius, wheel_length, wheel_mass, True, False)
wheel_fr.SetPos(chrono.ChVectorD(1, 0.25, 0))
wheel_fr.SetMaterialSurface(ChMaterialSurface.FRICTION_0_8)
wheel_fr.SetVisualizationType(chrono.ChVisualizationType_NONE)
system.Add(wheel_fr)


wheel_rl = chrono.ChBodyEasyCylinder(chrono.ChVectorD(-1, -0.25, 0), wheel_radius, wheel_length, wheel_mass, True, False)
wheel_rl.SetPos(chrono.ChVectorD(-1, -0.25, 0))
wheel_rl.SetMaterialSurface(ChMaterialSurface.FRICTION_0_8)
wheel_rl.SetVisualizationType(chrono.ChVisualizationType_NONE)
system.Add(wheel_rl)


wheel_rr = chrono.ChBodyEasyCylinder(chrono.ChVectorD(1, -0.25, 0), wheel_radius, wheel_length, wheel_mass, True, False)
wheel_rr.SetPos(chrono.ChVectorD(1, -0.25, 0))
wheel_rr.SetMaterialSurface(ChMaterialSurface.FRICTION_0_8)
wheel_rr.SetVisualizationType(chrono.ChVisualizationType_NONE)
system.Add(wheel_rr)


driver = ChDriver()
driver.SetVehicle(chassis)
driver.SetThrottle(0.5)
driver.SetSteering(0.0)
driver.SetBraking(0.0)



light1 = chrono.ChPointLight()
light1.SetPosition(chrono.ChVectorD(0, 1, 0))
light1.SetColor(chrono.ChColor(1, 1, 1))
light1.SetRadius(10)
system.Add(light1)

light2 = chrono.ChPointLight()
light2.SetPosition(chrono.ChVectorD(5, 1, 0))
light2.SetColor(chrono.ChColor(1, 0, 0))
light2.SetRadius(10)
system.Add(light2)


app = irrlicht.ChIrrApp(system, "Gator Simulation", irrlicht.dimension2d<irrlicht.uint16>(800, 600))
app.AddCamera(chrono.ChCamera())
app.GetCamera().SetPosition(chrono.ChVectorD(0, 1, 5))
app.GetCamera().SetTarget(chrono.ChVectorD(0, 0.5, 0))


while app.GetDevice().run():
    app.BeginScene()
    
    
    if app.GetDevice().isKeyDown(irrlicht.KEY_W):
        driver.SetThrottle(1.0)
    elif app.GetDevice().isKeyDown(irrlicht.KEY_S):
        driver.SetThrottle(0.0)
    if app.GetDevice().isKeyDown(irrlicht.KEY_A):
        driver.SetSteering(-0.5)
    elif app.GetDevice().isKeyDown(irrlicht.KEY_D):
        driver.SetSteering(0.5)
    else:
        driver.SetSteering(0.0)
    
    
    system.DoStepDynamics(1.0 / 60.0)
    
    
    app.GetCamera().SetPosition(chassis.GetPos() + chrono.ChVectorD(0, 1, 5))
    app.GetCamera().SetTarget(chassis.GetPos())
    
    
    app.DrawAll()
    app.EndScene()