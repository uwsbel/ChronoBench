import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()


truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)


bar = chrono.ChBody()
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVectorD(1, 1, 0.1))
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(bar)


gear1 = chrono.ChBody()
gear1.SetMass(1)
gear1.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.2))
gear1.SetPos(chrono.ChVectorD(-2, 0, 0))
sys.Add(gear1)

gear2 = chrono.ChBody()
gear2.SetMass(1)
gear2.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.2))
gear2.SetPos(chrono.ChVectorD(2, 0, 0))
sys.Add(gear2)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.Add(revolute_joint)


revolute_joint1 = chrono.ChLinkRevolute()
revolute_joint1.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVectorD(-2, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.Add(revolute_joint1)


revolute_joint2 = chrono.ChLinkRevolute()
revolute_joint2.Initialize(truss, gear2, chrono.ChCoordsysD(chrono.ChVectorD(2, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.Add(revolute_joint2)


gear_motor = chrono.ChLinkGear()
gear_motor.Initialize(gear1, gear2, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
gear_motor.Set_checkphase(False)
gear_motor.Set_enforce_realtime(True)
gear_motor.Set_ratio(-0.5) 
sys.Add(gear_motor)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Epicyclic Gears Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 6, -10))
vis.AddTypicalLights()


bar_mat = chrono.ChVisualMaterial()
bar_mat.SetKdTexture(chrono.GetChronoDataPath() + 'cubetexture.png')
bar.GetVisualShape(0).SetMaterial(0, bar_mat)

gear1_mat = chrono.ChVisualMaterial()
gear1_mat.SetKdTexture(chrono.GetChronoDataPath() + 'cubetexture.png')
gear1.GetVisualShape(0).SetMaterial(0, gear1_mat)

gear2_mat = chrono.ChVisualMaterial()
gear2_mat.SetKdTexture(chrono.GetChronoDataPath() + 'cubetexture.png')
gear2.GetVisualShape(0).SetMaterial(0, gear2_mat)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)