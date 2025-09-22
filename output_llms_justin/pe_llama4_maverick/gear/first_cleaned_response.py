import pychrono as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()


truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)


bar = chrono.ChBody()
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVector3d(1, 1, 0.1))
bar.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(bar)


gear1 = chrono.ChBody()
gear1.SetMass(1)
gear1.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
gear1.SetPos(chrono.ChVector3d(-1, 0, 0))
sys.Add(gear1)


gear2 = chrono.ChBody()
gear2.SetMass(1)
gear2.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
gear2.SetPos(chrono.ChVector3d(1, 0, 0))
sys.Add(gear2)


rev_joint_bar = chrono.ChLinkLockRevolute()
rev_joint_bar.Initialize(truss, bar, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.Add(rev_joint_bar)


rev_joint_gear1 = chrono.ChLinkLockRevolute()
rev_joint_gear1.Initialize(bar, gear1, chrono.ChCoordsysd(chrono.ChVector3d(-1, 0, 0), chrono.QUNIT))
sys.Add(rev_joint_gear1)


rev_joint_gear2 = chrono.ChLinkLockRevolute()
rev_joint_gear2.Initialize(truss, gear2, chrono.ChCoordsysd(chrono.ChVector3d(1, 0, 0), chrono.QUNIT))
sys.Add(rev_joint_gear2)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(gear1, gear2, False, chrono.ChVector3d(0, 0, 1), chrono.ChVector3d(0, 0, 1), -0.5)
sys.Add(gear_constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_PI / 2))  
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Epicyclic Gear Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


bar_shape = chrono.ChVisualShapeBox(0.1, 1, 0.1)
bar_shape.SetColor(chrono.ChColor(1, 0, 0))
bar.AddVisualShape(bar_shape)

gear1_shape = chrono.ChVisualShapeCylinder(0.2, 0.1)
gear1_shape.SetColor(chrono.ChColor(0, 1, 0))
gear1.AddVisualShape(gear1_shape)

gear2_shape = chrono.ChVisualShapeCylinder(0.2, 0.1)
gear2_shape.SetColor(chrono.ChColor(0, 0, 1))
gear2.AddVisualShape(gear2_shape)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()