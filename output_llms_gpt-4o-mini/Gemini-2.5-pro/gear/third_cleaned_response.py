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
mbody_train.GetVisualShape(0).SetMaterial(0, vis_mat) 


link_revoluteTT = chrono.ChLinkLockRevolute()                         
link_revoluteTT.Initialize(mbody_truss, mbody_train,                  
                           chrono.ChFramed(chrono.ChVector3d(0,0,0),  
                                           chrono.QUNIT))             
sys.AddLink(link_revoluteTT)                                          


gear_height = 0.5
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.Y_AXIS,             
                                        radA, gear_height,         
                                        1000, True, False, mat)    
sys.Add(mbody_gearA)                                               
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                    
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)              


mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                
mshaft_frame = chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)) 

                                                        


link_motor = chrono.ChLinkMotorRotationSpeed()                      

motor_frame_abs = chrono.ChFramed(mbody_gearA.GetPos(), chrono.QUNIT) 
link_motor.Initialize(mbody_gearA, mbody_truss, motor_frame_abs)
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))              
sys.AddLink(link_motor)                                             


interaxis12 = radA + radB                                           
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.Y_AXIS,              
                                        radB, 0.4,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearB)                                                
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))           
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)               



joint_frame_B_train = chrono.ChFramed(mbody_gearB.GetPos(), chrono.QUNIT)
link_revolute = chrono.ChLinkLockRevolute()                         
link_revolute.Initialize(mbody_gearB, mbody_train, joint_frame_B_train)
sys.AddLink(link_revolute)                                          




gear_AB_common_frame = chrono.ChFramed(chrono.VNULL, chrono.QUNIT)
link_gearAB = chrono.ChLinkLockGear()                                     
link_gearAB.Initialize(mbody_gearA, mbody_gearB, gear_AB_common_frame)    


q_shaft_rot = chrono.QuatFromAngleX(m.pi / 2)
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, q_shaft_rot))    
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, q_shaft_rot))    
link_gearAB.SetTransmissionRatio(radA / radB)                             
link_gearAB.SetEnforcePhase(True)                                         
sys.AddLink(link_gearAB)                                                  



radC = 2 * radB + radA                                                    

gear_BC_common_frame = chrono.ChFramed(chrono.VNULL, chrono.QUNIT) 
link_gearBC = chrono.ChLinkLockGear()                                     
link_gearBC.Initialize(mbody_gearB, mbody_truss, gear_BC_common_frame)    
link_
print("error happened with only start ```python")