import pychrono.core as chrono
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('')  
system = chrono.ChSystemNSC()


application = irr.ChIrrApp(system, 'CityBus Simulation', irr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0, 5, -15))
application.AddTypicalLights()


terrain = chrono.ChRigidTerrain(system)
terrain.SetTextureFilename('textures/concrete.jpg')  
terrain.Initialize(chrono.ChCoordsysD(irr.vector3d(0, 0, 0)))

ground = terrain.GetGroundBody()

groundShape = chrono.ChBoxShape(50, 0.1, 50)
groundShape.GetMaterial().SetFriction(0.8)
groundShape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.GetCollisionModel().AddShape(groundShape)
ground.GetCollisionModel().BuildModel()
ground.SetCollide(true)




bus = chrono.ChBodyEasyBox(4.0, 2.5, 8.0, 1000, True, True)
bus.SetPos(chrono.ChVectorD(0, 1.25, 0))
bus.SetMass(3000)
system.Add(bus)


bus_visual = chrono.ChModelFileShape()
bus_visual.SetFilename('models/citybus_mesh.obj')  
bus.GetAssets().push_back(bus_visual)






wheel_radius = 0.5
wheel_width = 0.3
wheel_positions = [
    chrono.ChVectorD(1.5, 0.5, 3.0),
    chrono.ChVectorD(-1.5, 0.5, 3.0),
    chrono.ChVectorD(1.5, 0.5, -3.0),
    chrono.ChVectorD(-1.5, 0.5, -3.0)
]
wheels = []
for pos in wheel_positions:
    wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 200, True, True)
    wheel.SetPos(bus.GetPos() + pos)
    wheel.SetMass(50)
    system.Add(wheel)
    
    wheel_visual = chrono.ChCylinderShape(wheel_radius, wheel_width)
    wheel.GetAssets().push_back(wheel_visual)
    wheels.append(wheel)


for wheel, pos in zip(wheels, wheel_positions):
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(wheel, bus, chrono.ChCoordsysD(bus.GetPos() + pos))
    system.Add(joint)






driver = irr.ChIrrGuiDriver(application)

steering = 0
throttle = 0
brake = 0


application.GetSceneManager().AddCamera(irr.ChCameraSceneNode(application.GetDevice().GetSceneManager(), bus, irr.vector3df(0, 2, -8)))


application.SetTimestep(1.0 / 50)  


while application.GetDevice().run():
    
    application.BeginScene()
    application.DrawAll()

    
    driver.BeginFrame()
    driver.Update()
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    brake = driver.GetBrake()

    
    
    
    print(f"Steering: {steering}, Throttle: {throttle}, Brake: {brake}")

    
    
    
    if throttle > 0:
        force = chrono.ChVectorD(0, 0, throttle * 1000)  
        bus.AccumulateForce(force, bus.GetPos(), chrono.VNULL)

    
    system.DoStepDynamics(1.0 / 50)

    
    application.EndScene()


application.GetDevice().close()