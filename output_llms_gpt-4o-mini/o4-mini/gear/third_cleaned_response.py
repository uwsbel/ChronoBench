import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math




sys = chrono.ChSystemNSC()


mat = chrono.ChMaterialSurfaceNSC()




radA = 1.5
radB = 3.5

mbody_truss = chrono.ChBodyEasyBox(
    15, 8, 2,         
    1000,             
    True,             
    False,            
    mat)
sys.Add(mbody_truss)
mbody_truss.SetBodyFixed(True)
mbody_truss.SetPos(chrono.ChVectorD(0, 0, 3))


vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))




mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVectorD(3, 0, 0))


rev_tt = chrono.ChLinkLockRevolute()
frame_tt = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
rev_tt.Initialize(mbody_truss, mbody_train, frame_tt)
sys.AddLink(rev_tt)




mbody_gearA = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,  
    radA,             
    0.5,              
    1000,
    True,
    False,
    mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))

mbody_gearA.SetRot(chrono.Q_from_AngX(math.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)


shaftA = chrono.ChCylinderShape(radA * 0.3,  
                                1.0)         
shaftA.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
mbody_gearA.AddVisualShape(
    shaftA,
    chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0),
                    chrono.Q_from_AngX(math.pi / 2)))


motor = chrono.ChLinkMotorRotationSpeed()
motor_frame = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
motor.Initialize(mbody_gearA, mbody_truss, motor_frame)
motor.SetSpeedFunction(chrono.ChFunction_Const(3.0))  
sys.AddLink(motor)




interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    radB,
    0.4,
    1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.Q_from_AngX(math.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)


rev_B = chrono.ChLinkLockRevolute()
frame_B = chrono.ChFrameD(chrono.ChVectorD(interaxis12, 0, 0), chrono.QUNIT)
rev_B.Initialize(mbody_train, mbody_gearB, frame_B)
sys.AddLink(rev_B)




gearAB = chrono.ChLinkLockGear()
gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD())
gearAB.SetFrameShaft1(
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),
                    chrono.Q_from_AngX(-math.pi / 2)))
gearAB.SetFrameShaft2(
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),
                    chrono.Q_from_AngX(-math.pi / 2)))
gearAB.SetTransmissionRatio(radA / radB)
gearAB.SetEnforcePhase(True)
sys.AddLink(gearAB)




radC = 2 * radB + radA
gearBC = chrono.ChLinkLockGear()
gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD())
gearBC.SetFrameShaft1(
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),
                    chrono.Q_from_AngX(-math.pi / 2)))
gearBC.SetFrameShaft2(
    chrono.ChFrameD(chrono.ChVectorD(0, 0, -4), chrono.QUNIT))
gearBC.SetTransmissionRatio(radB / radC)
gearBC.SetEpicyclic(True)
sys.AddLink(gearBC)




radD = 5.0
mbody_gearD = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Z,  
    radD,
    0.5,
    1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVectorD(-10, 0, -9))

mbody_gearD.SetRot(chrono.Q_from_AngZ(math.pi / 2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)


rev_D = chrono.ChLinkLockRevolute()

hinge_D = chrono.ChQuaternionD()  


rev_frame_D = chrono.ChFrameD(chrono.ChVectorD(-10, 0, -9), chrono.QUNIT)
rev_D.Initialize(mbody_truss, mbody_gearD, rev_frame_D)
sys.AddLink(rev_D)


gearAD = chrono.ChLinkLockGear()
gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFrameD())


gearAD.SetFrameShaft1(
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),
                    chrono.Q_from_AngX(-math.pi / 2)))
gearAD.SetFrameShaft2(
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),
                    chrono.QUNIT))
gearAD.SetTransmissionRatio(1.0)
gearAD.SetEnforcePhase(True)
sys.AddLink(gearAD)




radE = 2.0
mbody_pulleyE = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Z,
    radE,
    0.3,
    500, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVectorD(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.Q_from_AngZ(math.pi / 2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)


rev_E = chrono.ChLinkLockRevolute()
rev_frame_E = chrono.ChFrameD(chrono.ChVectorD(-10, -11, -9), chrono.QUNIT)
rev_E.Initialize(mbody_truss, mbody_pulleyE, rev_frame_E)
sys.AddLink(rev_E)


belt = chrono.ChLinkLockGear()
belt.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFrameD())
belt.SetFrameShaft1(
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
belt.SetFrameShaft2(
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
belt.SetTransmissionRatio(radD / radE)
belt.SetEnforcePhase(True)
sys.AddLink(belt)


belt_vis = chrono.ChBoxShape()

belt_vis.GetBoxGeometry().Size = chrono.ChVectorD(0.05, 5.5, 0.05)
mbody_truss.AddVisualShape(
    belt_vis,
    chrono.ChFrameD(chrono.ChVectorD(-10, -5.5, -9), chrono.QUNIT),
    vis_mat)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and Pulley Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(12, 15, -20))
vis.AddTypicalLights()




sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)