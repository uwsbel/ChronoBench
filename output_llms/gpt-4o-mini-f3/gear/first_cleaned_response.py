import pychrono as chrono                         
import pychrono.irrlicht as chronoirr             
import math as m                                  


sys = chrono.ChSystemNSC()                        


mat = chrono.ChContactMaterialNSC()               


radA = 2                                          
radB = 4                                          


mbody_truss = chrono.ChBodyEasyBox(20, 10, 2,     
                                    1000,          
                                    True,          
                                    False,         
                                    mat)           
sys.Add(mbody_truss)                              
mbody_truss.SetFixed(True)                        
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))    


vis_mat = chrono.ChVisualMaterial()                      
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  


mbody_train = chrono.ChBodyEasyBox(8, 1, 1,  
                                   1000,      
                                   True,      
                                   False,     
                                   mat)       
sys.Add(mbody_train)                             
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))   


con_revTR = chrono.ChLinkLockRevolute()                      
con_revTR.Initialize(mbody_truss, mbody_train,              
                     chrono.ChFramed(chrono.ChVector3d(0,0,0),  
                                     chrono.QUNIT))             
sys.Add(con_revTR)                                          


mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,     
                                        radA, 0.5,          
                                        1000, True, False, mat)  
sys.Add(mbody_gearA)                                        
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))             
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))         
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)       


mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.4, 13)                             
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     
                                                          chrono.QuatFromAngleX(chrono.CHPI_2)))  


interaxis12 = radA + radB                               
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,  
                                        radB, 0.4,          
                                        1000, True, False, mat)  
sys.Add(mbody_gearB)                                        
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -1))   
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))         
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)       


con_rev = chrono.ChLinkLockRevolute()                      
con_rev.Initialize(mbody_gearB, mbody_train,              
                   chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))  
sys.Add(con_rev)                                          


con_gearAB = chrono.ChLinkMotorRotationRadius()                      
con_gearAB.Initialize(mbody_gearA, mbody_gearB, interaxis12)         
sys.Add(con_gearAB)                                                  


con_gearAB.SetMotorFunction(chrono.ChFunctionConst(6))              


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