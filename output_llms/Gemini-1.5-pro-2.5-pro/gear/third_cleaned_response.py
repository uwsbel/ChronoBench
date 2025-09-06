import pychrono as chrono                         
import pychrono.irrlicht as chronoirr             
import math as m                                  


sys = chrono.ChSystemNSC()                        


mat = chrono.ChContactMaterialNSC()               


radA = 1.5                                        
radB = 3.5                                        
radC = 2 * radB + radA                            
radD = 5.0                                        
radE = 2.0                                        





mbody_truss = chrono.ChBodyEasyBox(25, 25, 2,      
                                   1000,          
                                   True,          
                                   False)         
sys.Add(mbody_truss)                              
mbody_truss.SetFixed(True)                        
mbody_truss.SetPos(chrono.ChVector3d(0, 0, -2))   


vis_mat_pink = chrono.ChVisualMaterial()
vis_mat_pink.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))
vis_mat_blue = chrono.ChVisualMaterial()
vis_mat_blue.SetKdTexture(chrono.GetChronoDataFile('textures/bluewhite.png'))
vis_mat_wood = chrono.ChVisualMaterial()
vis_mat_wood.SetKdTexture(chrono.GetChronoDataFile('textures/wood.jpg'))



mbody_truss.GetVisualShape(0).SetMaterial(vis_mat_wood)



mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,
                                   1000, True, False)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(0, 0, 0)) 


link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)



mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, 0))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2)) 

mbody_gearA.GetVisualShape(0).SetMaterial(vis_mat_pink)


link_motor = chrono.ChLinkMotorRotationSpeed()

link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3)) 
sys.AddLink(link_motor)


interaxis_AB = radA + radB

mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False)
sys.Add(mbody_gearB)
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))

mbody_gearB.GetVisualShape(0).SetMaterial(vis_mat_blue)


link_revolute_B = chrono.ChLinkLockRevolute()

link_revolute_B.Initialize(mbody_gearB, mbody_train,
                           chrono.ChFramed(chrono.ChVector3d(interaxis_AB, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute_B)




link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())

frame_shaft_Y = chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2))
link_gearAB.SetFrameShaft1(frame_shaft_Y)
link_gearAB.SetFrameShaft2(frame_shaft_Y)

link_gearAB.SetTransmissionRatio(-radA / radB)
sys.AddLink(link_gearAB)


link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(frame_shaft_Y) 
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QUNIT)) 

link_gearBC.SetCarrier(mbody_train)
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True) 
sys.AddLink(link_gearBC)




mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.5, 1000, True, False)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))

sys.Add(mbody_gearD)
mbody_gearD.GetVisualShape(0).SetMaterial(vis_mat_pink)


revolute_frame_D = chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), chrono.QuatFromAngleZ(m.pi/2))
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss, revolute_frame_D)
sys.AddLink(link_revoluteD)


link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(frame_shaft_Y)  
link_gearAD.SetFrameShaft2(frame_shaft_Y)  
link_gearAD.SetTransmissionRatio(-1.0) 
sys.AddLink(link_gearAD)


mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.5, 1000, True, False)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
sys.Add(mbody_pulleyE)
mbody_pulleyE.GetVisualShape(0).SetMaterial(vis_mat_blue)


revolute_frame_E = chrono.ChFramed(chrono.ChVector3d(-10, -11, -9), chrono.QuatFromAngleZ(m.pi/2))
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss, revolute_frame_E)
sys.AddLink(link_revoluteE)


link_beltDE = chrono.ChLinkLockGear()
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_beltDE.SetFrameShaft1(frame_shaft_Y) 
link_beltDE.SetFrameShaft2(frame_shaft_Y)

link_beltDE.SetTransmissionRatio(radD / radE)
sys.AddLink(link_beltDE)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Gear and Pulley Simulation')
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
    
    p1_start = posD + chrono.ChVector3d(0, 0, radD)
    p1_end   = posE + chrono.ChVector3d(0, 0, radE)
    p2_start = posD - chrono.ChVector3d(0, 0, radD)
    p2_end   = posE - chrono.ChVector3d(0, 0, radE)
    
    chronoirr.ChIrrTools.drawSegment(vis, p1_start, p1_end, chrono.ChColor(0.2, 0.2, 0.2))
    chronoirr.ChIrrTools.drawSegment(vis, p2_start, p2_end, chrono.ChColor(0.2, 0.2, 0.2))
    
    vis.EndScene()
    sys.DoStepDynamics(1e-3)