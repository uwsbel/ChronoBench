importchrono
import chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


application = chronoirr.ChIrrApp(system, "Simple Pendulum", chronoirr.dimension2du(800, 600))
application.AddTypicalIrrlichtLogo()
application.AddTypicalLights()
application.SetCameraPosition(chrono.ChVectorD(0, 3, 3))
application.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))


ground = chrono.ChBody()
ground.SetMass(0)  
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().AddBox(chrono.ChCollisionModel.BoxData(4, 1, 4))
ground.GetMaterialSurface().SetFriction(0.5)
system.AddBody(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1)  
pendulum.SetInertiaXX(chrono.ChVectorD(0.2, 0.2, 0.2))  
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))  


pendulum.GetCollisionModel().AddCylinder(chrono.ChCollisionModel.CylinderData(0.1, 0.2))
pendulum.GetVisualShape().AddCylinder(chrono.ChColor(0.8, 0, 0), 0.1, 0.2)

system.AddBody(pendulum)


joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0)))
system.AddLink(joint)


damping = chrono.ChDampingForce()
damping.SetCoefficient(0.1)
pendulum.AddForce(damping)


application.SetRenderMode(chronoirr.RENDER_MODE_OPENGL)
application.SetBackgroundColor(chrono.ChColor(0.1, 0.1, 0.1))
application.GetCamera().SetFov(60)
application.GetCamera().SetNearClip(0.1)


def log_state(system, time):
    pos = pendulum.GetPos()
    vel = pendulum.GetPos_dt()
    print(f"Time: {time:.2f}s | Position: {pos} | Velocity: {vel}")


application.SetTimestep(0.001)
application.SetPostStepCallback(log_state, 0.1)  

while application.Run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()