import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  






body_ground = chrono.ChBodyEasy()
body_ground.SetBodyFixed(True)
body_ground.SetPos(chrono.ChVectorD(0, -1, 0))
body_ground.SetShapeBoxes(0.5, 0.1, 1)
body_ground.SetCollide(True)
system.Add(body_ground)






body_pendulum = chrono.ChBodyEasy()
body_pendulum.SetMass(1.0)
body_pendulum.SetPos(chrono.ChVectorD(0, 1, 0))
body_pendulum.SetShapeCylinder(0.1, 0.5)
body_pendulum.SetCollide(True)


inertia = chrono.ChMatrix33()
inertia.SetDiagonal([0.0, 0.0, 0.0])
body_pendulum.SetInertia(inertia)

system.Add(body_pendulum)






joint_pendulum = chrono.ChLinkRevolute()
joint_pendulum.Initialize(body_ground, body_pendulum, chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0)))
system.Add(joint_pendulum)




body_pendulum.SetW(chrono.ChVectorD(0, 0, 1)) 






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simple Pendulum')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 2, -2))
vis.AddTypicalLights()






time_step = 0.01
time_end = 10.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    
    if int(system.GetChTime() / time_step) % 10 == 0:
        pos = body_pendulum.GetPos()
        vel = body_pendulum.GetW()
        print(f"Time: {system.GetChTime():.2f}, Pendulum Position: {pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}, Pendulum Velocity: {vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f}")