import pychrono as chrono
import pychrono.irrlicht as chronoirr
import os


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


ground_box = chrono.ChBoxShape()
ground_box.GetBoxGeometry().Size = chrono.ChVectorD(3, 0.1, 1)
ground_box.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddAsset(ground_box)


crank_radius = 0.15
rod_length = 0.4
piston_width = 0.1


crankshaft = chrono.ChBody()
crankshaft.SetPos(chrono.ChVectorD(0, 0.2, 0))
crankshaft.SetMass(5)
crankshaft.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(crankshaft)


crank_cyl = chrono.ChCylinderShape()
crank_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
crank_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(crank_radius, 0, 0)
crank_cyl.GetCylinderGeometry().rad = 0.02
crank_cyl.SetColor(chrono.ChColor(0.8, 0.1, 0.1))
crankshaft.AddAsset(crank_cyl)


rod = chrono.ChBody()
rod.SetMass(2)
rod.SetInertiaXX(chrono.ChVectorD(0.05, 0.5, 0.5))
system.Add(rod)


rod_box = chrono.ChBoxShape()
rod_box.GetBoxGeometry().Size = chrono.ChVectorD(rod_length/2, 0.02, 0.02)
rod_box.SetColor(chrono.ChColor(0.1, 0.5, 0.8))
rod.AddAsset(rod_box)


piston = chrono.ChBody()
piston.SetMass(3)
piston.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(piston)


piston_cyl = chrono.ChCylinderShape()
piston_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -piston_width/2, 0)
piston_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, piston_width/2, 0)
piston_cyl.GetCylinderGeometry().rad = 0.05
piston_cyl.SetColor(chrono.ChColor(0.8, 0.8, 0.1))
piston.AddAsset(piston_cyl)



crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(ground, crankshaft, 
                      chrono.ChCoordsysD(chrono.ChVectorD(0, 0.2, 0)))
system.Add(crank_joint)


crank_rod_joint = chrono.ChLinkLockRevolute()
crank_rod_joint.Initialize(crankshaft, rod,
                         chrono.ChCoordsysD(chrono.ChVectorD(crank_radius, 0.2, 0)))
system.Add(crank_rod_joint)


rod_piston_joint = chrono.ChLinkLockRevolute()
rod_piston_joint.Initialize(rod, piston,
                          chrono.ChCoordsysD(chrono.ChVectorD(crank_radius + rod_length, 0.2, 0)))
system.Add(rod_piston_joint)


piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Initialize(ground, piston,
                       chrono.ChCoordsysD(chrono.ChVectorD(0, 0.2, 0), 
                                         chrono.Q_from_AngZ(chrono.CH_C_PI_2)))
system.Add(piston_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, crankshaft, chrono.ChFrameD(chrono.ChVectorD(0, 0.2, 0)))
motor_speed = chrono.ChFunction_Const(chrono.CH_C_PI)  
motor.SetSpeedFunction(motor_speed)
system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()


camera = chronoirr.ChVectorD(1.5, 0.5, 1.2)
target = chrono.ChVectorD(0.5, 0.2, 0)
vis.AddCamera(camera, target)
vis.AddLight(chronoirr.SPointF(2, 2, 2), 5, chrono.ChColor(1, 1, 1))


vis.EnableBodyFrameDrawing(True)
vis.EnableLinkFrameDrawing(True)


if os.path.exists(chrono.GetChronoDataPath() + 'logo_chronoengine.png'):
    logo = vis.GetVideoDriver().getTexture(chrono.GetChronoDataFile('logo_chronoengine.png'))
    vis.AddLogo(logo, 10, 10)


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)