import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("/path/to/chrono/data/")  


sys = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


bar = chrono.ChBody()
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(bar)


gear1 = chrono.ChBody()
gear1.SetMass(1)
gear1.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
gear1.SetPos(chrono.ChVectorD(-2, 0, 0))
sys.Add(gear1)


gear2 = chrono.ChBody()
gear2.SetMass(1)
gear2.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
gear2.SetPos(chrono.ChVectorD(2, 0, 0))
sys.Add(gear2)


revolute1 = chrono.ChLinkRevolute()
revolute1.Initialize(bar, ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(revolute1)


revolute2 = chrono.ChLinkRevolute()
revolute2.Initialize(gear1, ground, chrono.ChCoordsysD(chrono.ChVectorD(-2, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(revolute2)


revolute3 = chrono.ChLinkRevolute()
revolute3.Initialize(gear2, bar, chrono.ChCoordsysD(chrono.ChVectorD(2, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(revolute3)


gear_motor = chrono.ChLinkGear()
gear_motor.Initialize(gear1, gear2, False, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)), chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
gear_motor.SetRatio(-0.5)  
sys.Add(gear_motor)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))  
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Epicyclic Gear Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 4, -6))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)