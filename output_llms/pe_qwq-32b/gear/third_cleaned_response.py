import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

sys = chrono.ChSystemNSC()
mat = chrono.ChContactMaterialNSC()
radA = 1.5
radB = 3.5


mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVectorD(0, 0, 3))

vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVectorD(3, 0, 0))


link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))
mbody_gearA.SetRot(chrono.Q_from_AngAxis(m.pi/2, chrono.ChVectorD(1,0,0)))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0), chrono.Q_from_AngAxis(m.pi/2, chrono.ChVectorD(1,0,0))))

link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))
sys.AddLink(link_motor)

interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.Q_from_AngAxis(m.pi/2, chrono.ChVectorD(1,0,0)))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train, chrono.ChFrameD(chrono.ChVectorD(interaxis12,0,0), chrono.QUNIT))
sys.AddLink(link_revolute)

link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD())
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.Q_from_AngAxis(-m.pi/2, chrono.ChVectorD(1,0,0))))
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.Q_from_AngAxis(-m.pi/2, chrono.ChVectorD(1,0,0))))
link_gearAB.SetTransmissionRatio(radA/radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

radC = 2*radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD())
link_gearBC.SetFrameShaft1(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.Q_from_AngAxis(-m.pi/2, chrono.ChVectorD(1,0,0))))
link_gearBC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0,0,-4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB/radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)


radD = 5.0
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVectorD(-10, 0, -9))
mbody_gearD.SetRot(chrono.Q_from_AngAxis(m.pi/2, chrono.ChVectorD(0,0,1)))  
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

link_revoluteD = chrono.ChLinkLockRevolute()
frameD = chrono.ChFrameD(chrono.ChVectorD(-10,0,-9), chrono.Q_from_AngAxis(m.pi/2, chrono.ChVectorD(1,0,0)))  
link_revoluteD.Initialize(mbody_gearD, mbody_truss, frameD)
sys.AddLink(link_revoluteD)

link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFrameD())
link_gearAD.SetTransmissionRatio(1.0)
link_gearAD.SetEnforcePhase(True)
sys.AddLink(link_gearAD)


radE = 2.0
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.3, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVectorD(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.Q_from_AngAxis(m.pi/2, chrono.ChVectorD(0,0,1)))  
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

link_revoluteE = chrono.ChLinkLockRevolute()
frameE = chrono.ChFrameD(chrono.ChVectorD(-10,-11,-9), chrono.Q_from_AngAxis(m.pi/2, chrono.ChVectorD(1,0,0)))
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss, frameE)
sys.AddLink(link_revoluteE)


link_belt = chrono.ChLinkBeltSynchro()
link_belt.Initialize(mbody_gearD, mbody_pulleyE)
sys.AddLink(link_belt)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears, Pulleys, and Belt')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(12,15,-20))
vis.AddTypicalLights()
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    posD = mbody_gearD.GetPos()
    posE = mbody_pulleyE.GetPos()
    vis.GetSystem().getVisualSystem().addLine(posD, posE, chrono.ChColor(1,1,0))
    
    vis.EndScene()
    sys.DoStepDynamics(1e-3)