importchrono
import chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


application = chronoirr.ChIrrApp(system, "Simple Pendulum", chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.SetCameraPosition(chrono.ChVectorD(0, 3, 5))
application.SetCameraRotation(chrono.ChVectorD(0, -45, 0))


ground = chrono.ChBody()
ground.SetMass(0)  
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().AddBox(chrono.ChCollisionModel(), chrono.ChVectorD(5, 1, 5))
ground.GetMaterialSurface().Set_texture(chrono.GetChronoDataFile("textures/grass.jpg"))
application.GetSystem().AddBody(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1)  
pendulum.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))  
pendulum.SetPos(chrono.ChVectorD(0, 2, 0))  


revolute = chrono.ChLinkRevolute()
revolute.Initialize(ground, pendulum, chrono.ChCoordsysD(
    chrono.ChVectorD(0, 0, 0),
    chrono.ChMatrix33D(chrono.ChQuaternionD(1, 0, 0, 0))
))
revolute.SetPivot(chrono.ChVectorD(0, 0, 0))
revolute.SetAxis(chrono.ChVectorD(1, 0, 0))
revolute.SetAngle(chrono.CH_C_45_DEG)  
application.GetSystem().AddLink(revolute)


pendulum.GetAssets().push_back(chrono.ChSphereShape(0.2))


constraint = chrono.ChLinkLockPointToLine()
constraint.Initialize(ground, pendulum, chrono.ChCoordsysD(
    chrono.ChVectorD(0, 2, 0),
    chrono.ChMatrix33D(chrono.ChQuaternionD(1, 0, 0, 0))
))
constraint.SetCfm(chrono.ChVectorD(0.001, 0.001, 0.001))
constraint.SetCfm_rot(chrono.ChVectorD(0.001, 0.001, 0.001))
application.GetSystem().AddLink(constraint)


application.SetTimestep(0.001)
application.BeginRun()

while application.GetDevice().run():
    application.BeginStep()
    application.DoStep()
    application.EndStep()
    
    
    if application.GetSystem().GetChTime() % 0.1 < 1e-6:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {application.GetSystem().GetChTime():.2f}s")
        print(f"Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        print(f"Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")
        print("------------------------")
    
    if application.GetDevice().keyPress(chrono.CH_KEY_ESCAPE):
        break

application.EndRun()
application.GetDevice().closeDevice()