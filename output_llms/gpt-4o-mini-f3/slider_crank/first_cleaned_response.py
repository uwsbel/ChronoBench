import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as chrr


system = chrono.ChSystemNSC()


mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.4)
mat.SetBounce(0.4)
system.SetContactMaterial(mat)


ground = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)  
system.Add(ground)


crankshaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 1, 0.4, 1000, True, True, mat)
crankshaft.SetPos(chrono.ChVector3d(-3, 0, 0))
crankshaft.SetRot(chrono.QuatFromAngleY(chrono.CH_PI_2))
system.Add(crankshaft)


flywheel = chrono.ChBodyEasyCircle(0.8, 1.2, 1000, True, True, mat)
flywheel.SetPos(chrono.ChVector3d(-3, 0.8, 0))
flywheel.SetRot(chrono.QuatFromAngleY(chrono.CH_PI_2))
system.Add(flywheel)


piston = chrono.ChBodyEasyBox(0.2, 0.2, 1, 1000, True, True, mat)
piston.SetPos(chrono.ChVector3d(-6, 0, 0))
system.Add(piston)


connrod = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000, True, True, mat)
connrod.SetPos(chrono.ChVector3d(-4.5, 0, 0))
system.Add(connrod)


my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(crankshaft, ground, chrono.ChFramed(chrono.ChVector3d(-3, 0, 0)))
my_motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI))
system.Add(my_motor)


joint_crank_connrod = chrono.ChLinkLockRevolute()
joint_crank_connrod.Initialize(crankshaft, connrod, chrono.ChFramed(chrono.ChVector3d(-4.5, 0, 0)))
system.Add(joint_crank_connrod)


joint_connrod_piston = chrono.ChLinkLockRevolute()
joint_connrod_piston.Initialize(connrod, piston, chrono.ChFramed(chrono.ChVector3d(-5.5, 0, 0)))
system.Add(joint_connrod_piston)


joint_piston_ground = chrono.ChLinkLockPrismatic()
joint_piston_ground.Initialize(piston, ground, chrono.ChFramed(chrono.ChVector3d(-6, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
system.Add(joint_piston_ground)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-slider motion')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 5.2), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()


time_step = 1e-3


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)