import pychrono as chrono
from pychrono import irrlicht
from pychrono.irrlicht import ChIrrApp


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))


ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetFixed(True)
system.Add(ground)


pendulum = chrono.ChBodyEasySphere(0.1, 1, True, True)
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))
pendulum.SetFixed(False)
system.Add(pendulum)


joint = chrono.ChLinkRevolute()
joint.SetName("revolute_joint")
joint.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, -1, 0), chrono.ChVectorD(0, 1, 0))
system.AddLink(joint)


application = irrlicht.ChIrrApp(system, "Pendulum Simulation", irrlicht.dimension2duint(800, 600))
application.AddCamera(chrono.ChCamera())
application.SetTimestep(1e-3)


application.AssetAddBody(ground)
application.AssetAddBody(pendulum)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(application.GetTimestep())

    
    if application.GetSimulationTime() % 0.1 == 0:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {application.GetSimulationTime()}, Position: {pos}, Velocity: {vel}")