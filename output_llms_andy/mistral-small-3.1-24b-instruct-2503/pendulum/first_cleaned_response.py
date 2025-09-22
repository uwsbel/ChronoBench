import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(10, 10, 0.5, 1000)
ground.SetPos(chrono.ChVectorD(0, -0.25, 0))
ground.SetBodyFixed(True)
system.Add(ground)


pendulum = chrono.ChBodyEasyCylinder(0.1, 0.5, 10, True, True, True, chrono.ChMaterialSurfaceNSC())
pendulum.SetPos(chrono.ChVectorD(0, 0, 0))
pendulum.SetMass(1)  
pendulum.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))  
system.Add(pendulum)


revolute = chrono.ChLinkRevolute()
revolute.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
system.Add(revolute)


application = chronoirr.ChIrrApp()
application.AddSystem(system)
application.SetWindowTitle("Simple Pendulum Simulation")
application.SetWindowSize(800, 600)
application.SetTryFullscreen(False)
application.SetShowInfo(true)
application.SetPaused(False)
application.SetTimestep(0.01)
application.SetTryFullscreen(False)


camera = chronoirr.ChIrrToolsCamera(application.GetDevice())
camera.SetTarget(chrono.ChVectorD(0, 0, 0))
camera.SetPos(chrono.ChVectorD(2, 2, 2))
application.AddCamera(camera)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    
    pos = pendulum.GetPos()
    vel = pendulum.GetPos_dt()
    print(f"Position: {pos}, Velocity: {vel}")


application.GetDevice().closeDevice()