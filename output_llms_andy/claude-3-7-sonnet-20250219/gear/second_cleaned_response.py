import pychrono as chrono                         
import pychrono.irrlicht as chronoirr             
import math as m                                  


sys = chrono.ChSystemNSC()                        


mat = chrono.ChMaterialSurfaceNSC()               


radA = 1.5                                        
radB = 3.5                                        


mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,      
                                   1000,          
                                   True,          
                                   False,         
                                   mat)           
sys.Add(mbody_truss)                              
mbody_truss.SetBodyFixed(True)                    
mbody_truss.SetPos(chrono.ChVectorD(0, 0, 3))     


vis_mat = chrono.ChVisualMaterial()                       
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  


mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,  
                                   1000,          
                                   True,          
                                   False,         
                                   mat)           
sys.Add(mbody_train)                              
mbody_train.SetPos(chrono.ChVectorD(3, 0, 0))     


link_revoluteTT = chrono.ChLinkLockRevolute()                         
link_revoluteTT.Initialize(mbody_truss, mbody_train,                  
                           chrono.ChCoordsysD(chrono.ChVectorD(0,0,0),  
                                           chrono.QUNIT))             
sys.AddLink(link_revoluteTT)                                          


mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             
                                        radA, 0.5,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearA)                                                
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))                      
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)               


mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0),     
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  


link_motor = chrono.ChLinkMotorRotationSpeed()                      
link_motor.Initialize(mbody_gearA, mbody_truss,                     
                      chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),    
                                      chrono.QUNIT))                
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))              
sys.AddLink(link_motor)                                             


interaxis12 = radA + radB                                           
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            
                                        radB, 0.4,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearB)                                                
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -2))            
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)               


link_revolute = chrono.ChLinkLockRevolute()                         
link_revolute.Initialize(mbody_gearB, mbody_train,                  
                         chrono.ChCoordsysD(chrono.ChVectorD(interaxis12, 0, 0), chrono.QUNIT))  
sys.AddLink(link_revolute)                                          



link_gearAB = chrono.ChLinkGear()                                     
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD())   
link_gearAB.SetRot1(chrono.ChQuaternionD(chrono.QUNIT))               
link_gearAB.SetRot2(chrono.ChQuaternionD(chrono.QUNIT))               
link_gearAB.SetTransmissionRatio(radA / radB)                         
sys.AddLink(link_gearAB)                                              



radC = 2 * radB + radA                                                
link_gearBC = chrono.ChLinkGear()                                     
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD())   
link_gearBC.SetRot1(chrono.ChQuaternionD(chrono.QUNIT))               
link_gearBC.SetRot2(chrono.ChQuaternionD(chrono.QUNIT))               
link_gearBC.SetPos2(chrono.ChVectorD(0, 0, -4))                       
link_gearBC.SetTransmissionRatio(-radB / radC)                        
sys.AddLink(link_gearBC)                                              


vis = chronoirr.ChVisualSystemIrrlicht()                        
vis.AttachSystem(sys)                                           
vis.SetWindowSize(1024, 768)                                    
vis.SetWindowTitle('Gears and pulleys')                         
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