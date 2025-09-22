import pychrono as chrono                         
import pychrono.irrlicht as chronoirr             
import math as m                                  


sys = chrono.ChSystemNSC()                        


mat = chrono.ChContactMaterialNSC()               


radA = 1.5                                        
radB = 3.5                                        
radC = 2 * radB + radA                                                    


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



radC = 2 * radB + radA                                                    
link_gearBC = chrono.ChLinkLockGear()                                     
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())       
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))        
link_gearBC.SetTransmissionRatio(radB / radC)                             
link_gearBC.SetEpicyclic(True)                                            
sys.AddLink(link_gearBC)                                                  


mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            
                                        radC, 0.5,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearD)                                                
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))                     
mbody_gearD.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)               


mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            
                                        2, 0.5,                  
                                        1000, True, False, mat)     
sys.Add(mbody_pulleyE)                                                
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))             
mbody_pulleyE.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)               


link_revoluteDE = chrono.ChLinkLockRevolute()                         
link_revoluteDE.Initialize(mbody_gearD, mbody_pulleyE,                  
                         chrono.ChFramed(chrono.ChVector3d(-10, 0, 0), chrono.QUNIT))  
sys.AddLink(link_revoluteDE)                                          


link_beltDE = chrono.ChLinkLockBelt()                                    
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())       
link_beltDE.SetRadius(2)                                                  
link_beltDE.SetAngle(chrono.CH_PI / 2)                                
link_beltDE.SetEnforceAngle(True)                                         
sys.AddLink(link_beltDE)                                                  


mshaft_shapeD = chrono.ChVisualShapeBevel(radC, 0.5, 0.5)             
mbody_gearD.AddVisualShape(mshaft_shapeD, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  

mshaft_shapeE = chrono.ChVisualShapePulley(2, 0.5, 0.5)             
mbody_pulleyE.AddVisualShape(mshaft_shapeE, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED) 


vis = chronoirr.ChVisualSystemIrrlicht()                        
vis.AttachSystem(sys)                                           
vis.SetWindowSize(1024, 768)                                    
vis.SetWindowTitle('Gears and pulleys with Bevel and Pulley')      
vis.Initialize()                                                
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) 
vis.AddSkyBox()                                                 
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                   
vis.AddTypicalLights()                                          


while vis.Run():                                    
    vis.BeginScene()                                
    vis.Render()                                    
    vis.EndScene()                                  
    sys.DoStepDynamics(1e-3)