import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
system.AddBody(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


box = chrono.ChVisualShapeBox(4, 0.5, 4)  
ground.AddVisualShape(box, chrono.ChFramed(chrono.ChVector3d(0, -0.25, 0)))  


pendulum = chrono.ChBody()
system.AddBody(pendulum)
pendulum.SetFixed(False)  
pendulum.EnableCollision(False)  
pendulum.SetMass(1)  
pendulum.SetInertiaXX(chrono.ChVector3d(0.0833, 0.6667, 0.6667))  


box = chrono.ChVisualShapeBox(1, 0.5, 0.5)  
box.SetColor(chrono.ChColor(0.8, 0, 0))  
pendulum.AddVisualShape(box, chrono.ChFramed(chrono.ChVector3d(-0.5, 0, 0)))  


pendulum.SetPos(chrono.ChVector3d(-1, 0, 0))  
pendulum.SetRot(chrono.QuatFromAngleY(0))  


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleY(0)))  
system.AddLink(joint)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Simple pendulum')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 2, 6), chrono.ChVector3d(0, 1, 0))  
vis.AddTypicalLights()  


time_step = 0.01


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  

    
    pos = pendulum.GetPos()  
    print(f'Pendulum position: {pos.x}, {pos.y}, {pos.z}')  
    print(f'Pendulum velocity: {pendulum.GetPosDt().x}, {pendulum.GetPosDt().y}, {pendulum.GetPosDt().z}')  

    system.DoStepDynamics(time_step)