import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()


truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)


bar = chrono.ChBody()
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVectorD(1, 1, 0.1))
bar.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(bar)


gear1 = chrono.ChBody()
gear1.SetMass(1)
gear1.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
gear1.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(gear1)


gear2 = chrono.ChBody()
gear2.SetMass(1)
gear2.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
gear2.SetPos(chrono.ChVectorD(2, 0, 0))
sys.Add(gear2)


rev_joint1 = chrono.ChLinkRevolute()
rev_joint1.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.Add(rev_joint1)


rev_joint2 = chrono.ChLinkRevolute()
rev_joint2.Initialize(bar, gear2, chrono.ChCoordsysD(chrono.ChVectorD(2, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.Add(rev_joint2)


rev_joint3 = chrono.ChLinkRevolute()
rev_joint3.Initialize(truss, gear1, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.Add(rev_joint3)


gear_motor = chrono.ChLinkGear()
gear_motor.Initialize(gear1, gear2, False, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y), chrono.ChCoordsysD(chrono.ChVectorD(2, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
gear_motor.Set_checkphase(False)
gear_motor.Set_enforce_ratio(True)
gear_motor.Set_ratio(-2) 
sys.Add(gear_motor)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
motor.SetSpeedFunction(chrono.ChFunction_Const(1)) 
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gear Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(4, 4, 4))
vis.AddTypicalLights()


gear1_mat = chrono.ChVisualMaterial()
gear1_mat.SetKdTexture(chrono.GetChronoDataPath() + 'aluminum.jpg')
gear1.AddVisualShape(chrono.ChSphereShape(0.5), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
gear1.GetVisualShape(0).SetMaterial(0, gear1_mat)

gear2_mat = chrono.ChVisualMaterial()
gear2_mat.SetKdTexture(chrono.GetChronoDataPath() + 'aluminum.jpg')
gear2.AddVisualShape(chrono.ChSphereShape(0.25), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
gear2.GetVisualShape(0).SetMaterial(0, gear2_mat)

bar_mat = chrono.ChVisualMaterial()
bar_mat.SetKdTexture(chrono.GetChronoDataPath() + 'steel.jpg')
bar.AddVisualShape(chrono.ChBoxShape(chrono.ChVectorD(2, 0.1, 0.1)), chrono.ChFrameD(chrono.ChVectorD(1, 0, 0)))
bar.GetVisualShape(0).SetMaterial(0, bar_mat)

truss_mat = chrono.ChVisualMaterial()
truss_mat.SetKdTexture(chrono.GetChronoDataPath() + 'concrete.jpg')
truss.AddVisualShape(chrono.ChBoxShape(chrono.ChVectorD(0.1, 0.1, 0.1)), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
truss.GetVisualShape(0).SetMaterial(0, truss_mat)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)