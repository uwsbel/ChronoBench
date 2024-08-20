import pychrono as chrono                         
import pychrono.irrlicht as chronoirr             
import math as m                                  


sys = chrono.ChSystemNSC()                        


mat = chrono.ChContactMaterialNSC()               


radA = 1.5                                        
radB = 3.5                                        
radC = 2 * radB + radA                            
radD = 5                                          
radE = 2                                          


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
                           chrono.ChFramed(chrono.ChVector3d(0,0,0),  
                                           chrono.QUNIT))             
sys.AddLink(link_revoluteTT)                                          


mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             
                                        radA, 0.5,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearA)                                                
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                     
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)               


mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  


link_motor = chrono.ChLinkMotorRotationSpeed()                      
link_motor.Initialize(mbody_gearA, mbody_truss,                     
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0),   
                                      chrono.QUNIT))                
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))              
sys.AddLink(link_motor)                                             


interaxis12 = radA + radB                                           
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            
                                        radB, 0.4,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearB)                                                
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))           
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)               


link_revolute = chrono.ChLinkLockRevolute()                         
link_revolute.Initialize(mbody_gearB, mbody_train,                  
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))  
sys.AddLink(link_revolute)                                          


link_gearAB = chrono.ChLinkLockGear()                                     
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())       
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    
link_gearAB.SetTransmissionRatio(radA / radB)                             
link_gearAB.SetEnforcePhase(True)                                         
sys.AddLink(link_gearAB)                                                  


link_gearBC = chrono.ChLinkLockGear()                                     
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())       
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))        
link_gearBC.SetTransmissionRatio(radB / radC)                             
link_gearBC.SetEpicyclic(True)                                            
sys.AddLink(link_gearBC)                                                  


mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             
                                        radD, 0.5,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearD)                                                
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))                     
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))                  
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)               


mshaft_shape = chrono.ChVisualShapeCylinder(radD * 0.3, 10)                                
mbody_gearD.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  


link_revoluteDD = chrono.ChLinkLockRevolute()                         
link_revoluteDD.Initialize(mbody_gearD, mbody_truss,                  
                           chrono.ChFramed(chrono.ChVector3d(-10, 0, -9),  
                                           chrono.QUNIT))             
sys.AddLink(link_revoluteDD)                                          


link_gearAD = chrono.ChLinkLockGear()                                     
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())       
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    
link_gearAD.SetTransmissionRatio(radA / radD)                             
link_gearAD.SetEnforcePhase(True)                                         
sys.AddLink(link_gearAD)                                                  


mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             
                                        radE, 0.5,                  
                                        1000, True, False, mat)     
sys.Add(mbody_pulleyE)                                                
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))                     
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))                  
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)               


mshaft_shape = chrono.ChVisualShapeCylinder(radE * 0.3, 10)                                
mbody_pulleyE.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  


link_revoluteEE = chrono.ChLinkLockRevolute()                         
link_revoluteEE.Initialize(mbody_pulleyE, mbody_truss,                  
                           chrono.ChFramed(chrono.ChVector3d(-10, -11, -9),  
                                           chrono.QUNIT))             
sys.AddLink(link_revoluteEE)                                          


link_beltDE = chrono.ChLinkSynchroBelt()                                     
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())       
link_beltDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    
link_beltDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    
link_beltDE.SetTransmissionRatio(radD / radE)                             
sys.AddLink(link_beltDE)                                                  


vis = chronoirr.ChVisualSystemIrrlicht()                        
vis.AttachSystem(sys)                                           
vis.SetWindowSize(1024, 768)                                    
vis.SetWindowTitle('Gears and pulleys')                         
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