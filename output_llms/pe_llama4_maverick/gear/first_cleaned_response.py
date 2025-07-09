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
gear1.SetMass(2)
gear1.SetInertiaXX(chrono.ChVector3d(2, 2, 0.2))
gear1.SetPos(chrono.ChVector3d(2, 0, 0))
sys.Add(gear1)

gear2 = chrono.ChBody()
gear2.SetMass(2)
gear2.SetInertiaXX(chrono.ChVector3d(2, 2, 0.2))
gear2.SetPos(chrono.ChVector3d(-2, 0, 0))
sys.Add(gear2)


rev_bar = chrono.ChLinkLockRevolute()
rev_bar.Initialize(bar, truss, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0)))
sys.Add(rev_bar)

rev_gear1 = chrono.ChLinkLockRevolute()
rev_gear1.Initialize(gear1, truss, chrono.ChCoordsysd(chrono.ChVector3d(2, 0, 0)))
sys.Add(rev_gear1)

rev_gear2 = chrono.ChLinkLockRevolute()
rev_gear2.Initialize(gear2, truss, chrono.ChCoordsysd(chrono.ChVector3d(-2, 0, 0)))
sys.Add(rev_gear2)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(gear1, gear2, truss, False, chrono.ChVector3d(0, 0, 1), chrono.ChVector3d(0, 0, 1), -0.5)
sys.Add(gear_constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1, bar, chrono.ChFrameD(chrono.ChVector3d(2, 0, 0)))
motor_fun = chrono.ChFunction_Const(chrono.CH_PI)  
motor.SetSpeedFunction(motor_fun)
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Epicyclic Gear Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 4, 6))
vis.AddTypicalLights()


bar_shape = chrono.ChVisualShapeCylinder(0.1, 4)
bar_shape.SetColor(chrono.ChColor(1, 0, 0))
bar.AddVisualShape(bar_shape)

gear1_shape = chrono.ChVisualShapeCylinder(0.2, 0.5)
gear1_shape.SetColor(chrono.ChColor(0, 1, 0))
gear1.AddVisualShape(gear1_shape)

gear2_shape = chrono.ChVisualShapeCylinder(0.4, 0.5)
gear2_shape.SetColor(chrono.ChColor(0, 0, 1))
gear2.AddVisualShape(gear2_shape)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()