import chrono as chrono
import irrlicht as irr


chrono.Chrono.SetDefaultOptions(irr.IrrAppOptions())
system = chrono.ChSystem()
system.Set_G_acc(chrono.ChVector(0, -9.81, 0))  


ground = system.AddRigidBody(chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True))
ground.SetPos(chrono.ChVector(0, 0, 0))
ground.SetBodyFixed(True)


turtlebot = system.Add(chrono.ChTurtlebot(system))
turtlebot.SetPos(chrono.ChVector(0, 0.5, 0))
turtlebot.SetRot(chrono.ChQuaternion(1, 0, 0, 0))


vis = irr.ChVisualSystemIrrlicht()
vis.SetUpSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowCaption("Turtlebot Simulation")
vis.SetCameraPos(chrono.ChVector(0, 10, -15))
vis.SetCameraTarget(chrono.ChVector(0, 0.5, 0))
vis.SetChaseCamera(True, 1.5)
vis.SetShowInfos(True)
vis.SetShowAxes(True)
vis.SetShowCoordinateSystem(True)
vis.SetLightingEnabled(True)
vis.SetAmbientLight(chrono.ChColor(0.3f, 0.3f, 0.3f))
vis.SetDiffuseLight(chrono.ChColor(0.7f, 0.7f, 0.7f))
vis.SetSpecularLight(chrono.ChColor(0.1f, 0.1f, 0.1f))


t = 0
dt = 1.0 / 60.0
while vis.Run():
    system.DoStepDynamics(dt)
    t += dt

    if t > 2 and t < 4:
        turtlebot.SetMotorSpeeds(1, 0)  
    elif t > 6 and t < 8:
        turtlebot.SetMotorSpeeds(0, 1)  
    else:
        turtlebot.SetMotorSpeeds(0, 0)

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

vis.Close()