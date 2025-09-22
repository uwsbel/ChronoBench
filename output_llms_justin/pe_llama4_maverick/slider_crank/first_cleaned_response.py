import pychrono as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


crank = chrono.ChBody()
crank.SetPos(chrono.ChVector3d(0, 0, 0))
crank.SetMass(1)
crank.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
sys.Add(crank)


rod = chrono.ChBody()
rod.SetPos(chrono.ChVector3d(2, 0, 0))
rod.SetMass(1)
rod.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
sys.Add(rod)


piston = chrono.ChBody()
piston.SetPos(chrono.ChVector3d(4, 0, 0))
piston.SetMass(1)
piston.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
sys.Add(piston)


rev_joint_crank = chrono.ChLinkLockRevolute()
rev_joint_crank.Initialize(ground, crank, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0)))
sys.Add(rev_joint_crank)


rev_joint_rod = chrono.ChLinkLockRevolute()
rev_joint_rod.Initialize(crank, rod, chrono.ChCoordsysD(chrono.ChVector3d(1, 0, 0)))
sys.Add(rev_joint_rod)


rev_joint_piston = chrono.ChLinkLockRevolute()
rev_joint_piston.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVector3d(3, 0, 0)))
sys.Add(rev_joint_piston)


pris_joint_piston = chrono.ChLinkLockPrismatic()
pris_joint_piston.Initialize(piston, ground, chrono.ChCoordsysD(chrono.ChVector3d(4, 0, 0), chrono.Q_from_AngX(chrono.CH_PI/2)))
sys.Add(pris_joint_piston)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, ground, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0)))
sys.Add(motor)


motor_fun = chrono.ChFunction_Const(chrono.CH_PI)  
motor.SetSpeedFunction(motor_fun)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()