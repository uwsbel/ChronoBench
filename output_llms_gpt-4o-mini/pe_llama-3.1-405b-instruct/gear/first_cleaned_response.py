import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


chrono.SetChronoDataPath('/path/to/data')


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()


truss_body = chrono.ChBody()
truss_body.SetBodyFixed(True)
truss_shape = chrono.ChVisualShapeBox(chrono.ChVectorD(2, 0.5, 0.5))
truss_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
truss_body.AddVisualShape(truss_shape)
sys.Add(truss_body)


bar_body = chrono.ChBody()
bar_body.SetPos(chrono.ChVectorD(0, 1, 0))
bar_shape = chrono.ChVisualShapeCylinder(0.1, 2)
bar_shape.SetColor(chrono.ChColor(1, 0, 0))
bar_body.AddVisualShape(bar_shape)
sys.Add(bar_body)


gear1_body = chrono.ChBody()
gear1_body.SetPos(chrono.ChVectorD(-1, 1, 0))
gear1_shape = chrono.ChVisualShapeCylinder(0.5, 0.5)
gear1_shape.SetColor(chrono.ChColor(0, 1, 0))
gear1_body.AddVisualShape(gear1_shape)
sys.Add(gear1_body)

gear2_body = chrono.ChBody()
gear2_body.SetPos(chrono.ChVectorD(1, 1, 0))
gear2_shape = chrono.ChVisualShapeCylinder(0.5, 0.5)
gear2_shape.SetColor(chrono.ChColor(0, 0, 1))
gear2_body.AddVisualShape(gear2_shape)
sys.Add(gear2_body)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar_body, gear1_body, chrono.ChFrameD(chrono.ChVectorD(0, 1, 0)))
sys.Add(motor)
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(bar_body, gear2_body, chrono.ChFrameD(chrono.ChVectorD(1, 1, 0)))
sys.Add(rev_joint)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()