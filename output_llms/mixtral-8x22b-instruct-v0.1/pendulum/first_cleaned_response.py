import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data")  
chrono.SetChronoDataPath("/path/to/irrlicht/data")  


physics_system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(width=10, height=0.1, length=10, density=1000)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.SetBodyFixed(True)
physics_system.Add(ground)


mass = 1.0
inertia = chrono.ChVectorD(0.01, 0.01, 0.01)
radius = 0.1
pendulum = chrono.ChBodyEasySphere(mass, radius, True, True, inertia)
pendulum.SetPos(chrono.ChVectorD(0, 0.5, 0))
physics_system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
physics_system.Add(joint)


pendulum.SetPos_dt(chrono.ChVectorD(0, -1, 0))


visualization_system = chronoirr.ChIrrApp(physics_system, "Simple Pendulum", chronoirr.dimension2du(800, 600), True, True)


visualization_system.AddTypicalLogo()
visualization_system.AddTypicalSky()
visualization_system.AddTypicalLights()
visualization_system.AddTypicalCamera(chrono.ChVectorD(0, 0, -1))
visualization_system.AddLightWithShadow(chrono.ChVectorD(1, 1, 0), 1, 50, 2, 50, 50)


ground_shape = chronoirr.ChIrrCylinderShape(ground, 10, 0.1)
ground_shape.SetColor(chronoirr.ChColor(0.5, 0.5, 0.5))
visualization_system.AddAsset(ground_shape)

pendulum_shape = chronoirr.ChIrrCylinderShape(pendulum, radius, 0.01)
pendulum_shape.SetColor(chronoirr.ChColor(0.8, 0.2, 0.2))
visualization_system.AddAsset(pendulum_shape)


while visualization_system.Run():
    physics_system.DoStepDynamics(0.01)
    visualization_system.BeginScene()
    visualization_system.DrawAll()
    visualization_system.EndScene()

    
    position = pendulum.GetPos()
    velocity = pendulum.GetPos_dt()
    print(f"Position: {position.x}, {position.y}, {position.z}")
    print(f"Velocity: {velocity.x}, {velocity.y}, {velocity.z}")