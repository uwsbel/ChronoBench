import pychrono as chrono                         
import pychrono.irrlicht as chronoirr             
import math as m                                  


sys = chrono.ChSystemNSC()                        


mat = chrono.ChContactMaterialNSC()               


radA = 1.5                                        
radB = 3.5                                        


mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,      
                                   1000,          
                                   True,          
                                   False,         
                                   mat)           
sys.Add(mbody_truss)                              
mbody_truss.SetFixed(True)                        
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))    


vis_mat = chrono.ChVisualMaterial()                       
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  


mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,  
                                   1000,          
                                   True,          
                                   False,         
                                   mat)           
sys.Add(mbody_train)                              
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))    


link_revoluteTT = chrono.ChLinkLockRevolute()                         
link_revoluteTT.Initialize(mbody_truss, mbody_train,                  
                           chrono.ChFrameD(chrono.ChVector3d(0,0,3),  
                                           chrono.Q_from_AngAxis(0, chrono.ChVector3D(0,0,1))))             
sys.AddLink(link_revoluteTT)                                          


mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             
                                        radA, 0.5,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearA)                                                
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)


gearA_vis_shape = chrono.ChVisualShapeCylinder(radA, 0.5)
mbody_gearA.AddVisualShape(gearA_vis_shape, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0), chrono.Quat(1,0,0,0)))



link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChFrameD(chrono.ChVector3d(0, 0, -1), chrono.Q_from_AngX(0)))
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))
sys.AddLink(link_motor)


interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)


link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChFrameD(chrono.ChVector3d(interaxis12, 0, -1), chrono.Q_from_AngX(0)))
sys.AddLink(link_revolute)



link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB,
                       chrono.ChFrameD(chrono.ChVector3d(0, 0, -1), chrono.Q_from_AngX(0)))
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.ChVector3d(0,0, -1), chrono.Q_from_AngX(0)))
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.ChVector3d(0,0, -1), chrono.Q_from_AngX(0)))
link_gearAB.SetTransmissionRatio(1.0)  
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)


radC = 2 * radB + radA



radD = 5.0

mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radD, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearD)

mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))

qDrotation = chrono.Q_from_AngleAxis(m.pi / 2, chrono.ChVector3D(0, 0, 1))
mbody_gearD.SetRot(qDrotation)
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)


gearD_vis_shape = chrono.ChVisualShapeCylinder(radD, 0.5)
mbody_gearD.AddVisualShape(gearD_vis_shape, chrono.ChFrameD(chrono.ChVector3d(0,0,0), qDrotation))


link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_truss, mbody_gearD,
                          chrono.ChFrameD(chrono.ChVector3d(-10, 0, -9), chrono.Q_from_AngAxis(0, chrono.ChVector3D(0,0,1))))
sys.Add(link_revoluteD)


link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD,
                       chrono.ChFrameD(chrono.ChVector3d(0,0,-1), chrono.Q_from_AngX(0)))
link_gearAD.SetFrameShaft1(chrono.ChFrameD(chrono.ChVector3d(0,0,-1), chrono.Q_from_AngX(0)))
link_gearAD.SetFrameShaft2(chrono.ChFrameD(chrono.ChVector3d(0,0,-1), chrono.Q_from_AngX(0)))
link_gearAD.SetTransmissionRatio(1.0)
link_gearAD.SetEnforcePhase(True)
sys.AddLink(link_gearAD)


pulley_radius = 2.0

mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, pulley_radius, 0.3, 1000, True, False, mat)
sys.Add(mbody_pulleyE)

mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))

qErotation = chrono.Q_from_AngleAxis(m.pi / 2, chrono.ChVector3D(0, 0, 1))
mbody_pulleyE.SetRot(qErotation)

pulley_vis_shape = chrono.ChVisualShapeCylinder(pulley_radius, 0.3)
mbody_pulleyE.AddVisualShape(pulley_vis_shape, chrono.ChFrameD(chrono.ChVector3d(0,0,0), qErotation))


link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_truss, mbody_pulleyE,
                          chrono.ChFrameD(chrono.ChVector3d(-10, -11, -9), chrono.Q_from_AngAxis(0, chrono.ChVector3D(0,0,1))))
sys.Add(link_revoluteE)


link_belt = chrono.ChLinkBelt()


pointD = chrono.ChVector3d(0, 0, 0)  
pointE = chrono.ChVector3d(0, 0, 0)  

link_belt.Initialize(mbody_gearD, mbody_pulleyE,
                     chrono.ChFrameD(chrono.ChVector3d(0, 0, 0),
                                       chrono.Q_identity()),
                     chrono.ChFrameD(chrono.ChVector3d(0, 0, 0),
                                       chrono.Q_identity()))

link_belt.SetTension(10)
sys.Add(link_belt)







def add_belt_visual(vis, body1, body2):
    pos1 = body1.GetPos()
    pos2 = body2.GetPos()
    vis.AddLine(pos1, pos2, chrono.ChColor(1, 0, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys with belt')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)


while vis.Run():
    
    add_belt_visual(vis, mbody_gearD, mbody_pulleyE)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)