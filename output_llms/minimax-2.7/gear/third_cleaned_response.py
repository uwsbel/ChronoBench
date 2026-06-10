import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m




sys = chrono.ChSystemNSC()
mat = chrono.ChContactMaterialNSC()          




radA = 1.5
radB = 3.5
radD = 5.0
radE = 2.0

interaxisAB = radA + radB                    




vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))




mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,
                                    1000,          
                                    True,          
                                    False,         
                                    mat)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))
mbody_truss.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(mbody_truss)




mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,
                                    1000,
                                    True,
                                    False,
                                    mat)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))
mbody_train.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(mbody_train)


link_rev_truss_train = chrono.ChLinkLockRevolute()
link_rev_truss_train.Initialize(
    mbody_truss,
    mbody_train,
    chrono.ChFramed(chrono.ChVector3d(3, 0, 0), chrono.QUNIT))   
sys.AddLink(link_rev_truss_train)




mbody_gearA = chrono.ChBodyEasyCylinder(
    chrono.ChVector3d(0, 1, 0),            
    radA, 0.5,
    1000, True, False, mat)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))   
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(mbody_gearA)


shaftA = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(shaftA,
                            chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                                            chrono.QuatFromAngleX(chrono.CH_PI_2)))


link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(
    mbody_gearA,
    mbody_truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, -1), chrono.QUNIT))   
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3.0))
sys.AddLink(link_motor)




mbody_gearB = chrono.ChBodyEasyCylinder(
    chrono.ChVector3d(0, 1, 0),
    radB, 0.4,
    1000, True, False, mat)
mbody_gearB.SetPos(chrono.ChVector3d(interaxisAB, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(mbody_gearB)


link_rev_B_train = chrono.ChLinkLockRevolute()
link_rev_B_train.Initialize(
    mbody_gearB,
    mbody_train,
    chrono.ChFramed(chrono.ChVector3d(interaxisAB, 0, 0), chrono.QUNIT))
sys.AddLink(link_rev_B_train)


link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL,
                                           chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL,
                                           chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)




radC = 2 * radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL,
                                            chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4),
                                            chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)




mbody_gearD = chrono.ChBodyEasyCylinder(
    chrono.ChVector3d(0, 1, 0),            
    radD, 0.5,
    1000, True, False, mat)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))

mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(mbody_gearD)


shaftD = chrono.ChVisualShapeCylinder(radD * 0.2, 10)
mbody_gearD.AddVisualShape(shaftD,
                            chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                                            chrono.QuatFromAngleX(chrono.CH_PI_2)))



link_rev_D_truss = chrono.ChLinkLockRevolute()
link_rev_D_truss.Initialize(
    mbody_truss,
    mbody_gearD,
    chrono.ChFramed(chrono.ChVector3d(-10, 0, -9),
                    chrono.QuatFromAngleY(chrono.CH_PI_2)))   
sys.AddLink(link_rev_D_truss)


link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.VNULL,
                                           chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.VNULL,
                                           chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetTransmissionRatio(1.0)            
link_gearAD.SetEnforcePhase(False)
sys.AddLink(link_gearAD)




mbody_pulleyE = chrono.ChBodyEasyCylinder(
    chrono.ChVector3d(0, 1, 0),
    radE, 0.4,
    1000, True, False, mat)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))   
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(mbody_pulleyE)


shaftE = chrono.ChVisualShapeCylinder(radE * 0.25, 10)
mbody_pulleyE.AddVisualShape(shaftE,
                              chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                                              chrono.QuatFromAngleX(chrono.CH_PI_2)))


link_rev_E_truss = chrono.ChLinkLockRevolute()
link_rev_E_truss.Initialize(
    mbody_truss,
    mbody_pulleyE,
    chrono.ChFramed(chrono.ChVector3d(-10, -11, -9),
                    chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddLink(link_rev_E_truss)




belt = chrono.ChLinkLockBelt(radD, radE)          
belt.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())

belt.SetFrameShaft1(chrono.ChFramed(chrono.VNULL,
                                   chrono.QuatFromAngleX(-m.pi / 2)))
belt.SetFrameShaft2(chrono.ChFramed(chrono.VNULL,
                                   chrono.QuatFromAngleX(-m.pi / 2)))
sys.AddLink(belt)





belt_line = chrono.ChVisualShapeLine()
belt_line.SetLineThickness(0.05)                
vis.AddVisualShape(belt_line, chrono.ChFramed())  




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears, pulley and synchro‑belt')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)




while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    posD = mbody_gearD.GetPos()
    posE = mbody_pulleyE.GetPos()
    belt_line.SetPoint1(chrono.ChVector3d(posD.x, posD.y, posD.z))
    belt_line.SetPoint2(chrono.ChVector3d(posE.x, posE.y, posE.z))

    vis.EndScene()

    sys.DoStepDynamics(1e-3)