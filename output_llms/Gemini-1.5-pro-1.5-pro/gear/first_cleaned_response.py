import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


truss = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile('models/truss.obj'),  
                              1000,  
                              True,  
                              True)  
truss.SetBodyFixed(True)
system.Add(truss)


bar_length = 0.5
bar = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, bar_length/2, 0.02, 1000)
bar.SetPos(chrono.ChVectorD(0, 0, 0.2))
bar.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  
system.Add(bar)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.2), chrono.Q_from_AngY(0)))
system.AddLink(revolute_joint)




radius1 = 0.1
radius2 = 0.2
gear1 = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radius1, 0.01, 1000)
gear1.SetPos(chrono.ChVectorD(0.2, 0, 0.2))
system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radius2, 0.01, 1000)
gear2.SetPos(chrono.ChVectorD(0.2 + radius1 + radius2, 0, 0.2))
system.Add(gear2)




gear_constraint = chrono.ChLinkGearCylindrical()
gear_constraint.Initialize(gear1, gear2, 
                         chrono.ChFrameD(chrono.ChVectorD(0.2+radius1, 0, 0.2), chrono.Q_from_AngY(0)),
                         chrono.ChFrameD(chrono.ChVectorD(0.2+radius1, 0, 0.2), chrono.Q_from_AngY(0)),
                         radius1, radius2)

gear_constraint.SetTransmissionRatio(radius2/radius1)
system.AddLink(gear_constraint)




motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1,                               
                 chrono.ChFrameMovingD(gear1.GetFrame_REF_to_abs()))
motor_speed = chrono.ChFunction_Const(2 * chrono.CH_C_PI) 
motor.SetSpeedFunction(motor_speed)
system.AddLink(motor)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 1), chrono.ChVectorD(0, 0, 0.2))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)