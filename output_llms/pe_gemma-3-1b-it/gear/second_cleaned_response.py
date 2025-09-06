import pychrono as chrono                         
import pychrono.irrlicht as chronoirr             
import math as m                                  


sys = chrono.ChSystemNSC()                        


mat = chrono.ChContactMaterialNSC()               


radA = 2                                          
radB = 4                                          


mbody_truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False, mat)
sys.Add(mbody_truss)                              

mbody_truss.SetFixed(True)                        
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 0))    
mbody_truss.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2)) 


rev_joint = chrono.ChLinkLockRevolute()                         
rev_joint.Initialize(mbody_truss, mbody_train,                  
                         chrono.ChFramed(chrono.ChVector3d(0, 0, 0),     
                                           chrono.QUNIT))             

sys.AddLink(rev_joint)                                          


spherical_joint = chrono.ChLinkLockSpherical()                         
spherical_joint.Initialize(mbody_gearA, mbody_train,                  
                           chrono.ChFramed(chrono.ChVector3d(0, 0, 0),    
                                           chrono.QUNIT))             

sys.AddLink(spherical_joint)                                          


univ_joint = chrono.ChLinkUniversal()                                     
univ_joint.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed(chrono.ChVector3d(0, 0, 0),  
                                           chrono.QUNIT))   

sys.AddLink(univ_joint)                                          


motor = chrono.ChLinkMotorRotationSpeed()                         
motor.Initialize(mbody_gearB, mbody_train,                  
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0),   
                                      chrono.QUNIT))             

sys.AddLink(motor)                                                  


mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             
                                        radA, 0.5,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearA)                                                
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                     

mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2)) 

mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)               


mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.4, 13)                                
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),  
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))   


link_motor = chrono.ChLinkMotorRotationSpeed()                                     
link_motor.Initialize(mbody_gearB, mbody_train,                  
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, -1),    
                                           chrono.QUNIT))             

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
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, -1),    
                                           chrono.QUNIT))             

sys.AddLink(link_revolute)                                          



link_gearAB = chrono.ChLinkLockGear()                                     
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, -4),  
                                           chrono.QUNIT))   

sys.AddLink(link_gearAB)                                          


vis = chronoirr.ChVisualSystemIrrlicht()                        
vis.AttachSystem(sys)                                           
vis.SetWindowSize(1024, 768)                                    
vis.SetWindowTitle('Gears and pulleys')                         
vis.Initialize()                                                
vis.AddSkyBox()                                                 
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                   
vis.AddTypicalLights()