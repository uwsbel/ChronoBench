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
                           chrono.ChFrameD(chrono.ChVector3d(0, 0, 3), chrono.Q_from_AngX(0)))
sys.AddLink(link_revoluteTT)


mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                        radA, 0.5,
                                        1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.Q_from_AngX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)


mshaft_shape_A = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape_A, chrono.ChFrameD(chrono.ChVector3d(0, 3.5, 0),
                                                          chrono.Q_from_AngX(m.pi / 2)))


link_motorA = chrono.ChLinkMotorRotationSpeed()
link_motorA.Initialize(mbody_gearA, mbody_truss,
                       chrono.ChFrameD(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(0)))
link_motorA.SetSpeedFunction(chrono.ChFunction_Const(3))
sys.Add(link_motorA)


interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                        radB, 0.4,
                                        1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.Q_from_AngX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)


link_revoluteB = chrono.ChLinkLockRevolute()
link_revoluteB.Initialize(mbody_gearB, mbody_train,
                          chrono.ChFrameD(chrono.ChVector3d(interaxis12, 0, -1), chrono.Q_from_AngX(0)))
sys.AddLink(link_revoluteB)


link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD(chrono.VNULL))
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi/2)))
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi/2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)


radC = 2 * radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss,
                       chrono.ChFrameD(chrono.ChVector3d(0, 0, -4), chrono.Q_from_AngX(0)))
link_gearBC.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi/2)))
link_gearBC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVector3d(0, 0, -4), chrono.Q_from_AngX(0)))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)


radD = 5
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z,
                                        radD, 0.5,
                                        1000, True, False, mat)
sys.Add(mbody_gearD)

mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))

mbody_gearD.SetRot(chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1)))

mshaft_shape_D = chrono.ChVisualShapeCylinder(radD * 0.3, 10)
mbody_gearD.AddVisualShape(mshaft_shape_D, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0),
                                                          chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1))))

link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                          chrono.ChFrameD(chrono.ChVector3d(-10, 0, -9), chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1))))
sys.Add(link_revoluteD)


radE = 2
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z,
                                          radE, 0.2,
                                          1000, True, False, mat)
sys.Add(mbody_pulleyE)

mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))

mbody_pulleyE.SetRot(chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1)))

mshaft_shape_E = chrono.ChVisualShapeCylinder(radE * 0.3, 10)
mbody_pulleyE.AddVisualShape(mshaft_shape_E, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0),
                                                          chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1))))

link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                          chrono.ChFrameD(chrono.ChVector3d(-10, -11, -9), chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1))))
sys.Add(link_revoluteE)



belt_thickness = 0.05
belt_color = chrono.ChColorAsset(0.8, 0.2, 0.2)  


belt_shape = chrono.ChLineShape()

start_point = mbody_gearD.GetPos()

end_point = mbody_pulleyE.GetPos()





mid_point = (start_point + end_point) * 0.5

dir_vec = end_point - start_point
length = dir_vec.Length()

belt_cylinder = chrono.ChBodyEasyCylinder(belt_thickness, length,
                                         0, True, False, chrono.ChMaterialSurface())

belt_cylinder.SetPos(mid_point)

axis = dir_vec.GetNormalized()

rotation = chrono.Q_from_AngAxis(m.pi/2, axis.Cross(chrono.ChVector3d(0, 0, 1))) if axis.Dot(chrono.ChVector3d(0,0,1)) < 0.99 else chrono.Q_from_AngAxis(0, axis)
belt_cylinder.SetRot(rotation)

belt_shape_asset = chrono.ChVisualShapeCylinder(belt_thickness, length)
belt_shape_asset.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
belt_cylinder.AddVisualShape(belt_shape_asset)


sys.Add(belt_cylinder)














vis = chronoirr.ChVisualSystemIrrlicht()                        
vis.AttachSystem(sys)                                           
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys with additional components')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)