import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
system.Add(ground)


rover = chrono.ChBodyEasyBox(1, 1, 2, 100)
rover.SetPos(chrono.ChVectorD(0, 1, 0))
system.Add(rover)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(rover, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(motor)


application = irr.ChIrrApp(system, 'Curiosity Rover Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0, 3, -6))
application.AddTypicalLights()
application.SetTimestep(0.01)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()


application.GetSystem().Clear()