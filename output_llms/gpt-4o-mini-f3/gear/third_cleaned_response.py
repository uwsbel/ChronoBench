import pychrono as chrono                         
import pychrono.irrlicht as chronoirr             
import math as m                                  


sys = chrono.ChSystemNSC()                        


mat = chrono.ChContactMaterialNSC()               


radA = 1.5                                        
radB = 3.5                                        




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
	
	if link_pulleyDE.GetBelt():	
	    link_pulleyDE.GetBelt().SetFixed(True)  
	    vis.AddTriangleShape(link_pulleyDE.GetBelt().GetShape(),chrono.ChColor(0.8,0.2,0.2))  
    vis.EndScene()                                  
    sys.DoStepDynamics(1e-3)