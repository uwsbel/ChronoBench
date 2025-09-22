import pychrono as chrono                         
import pychrono.irrlicht as chronoirr             
import math as m                                  


sys = chrono.ChSystemNSC()                        


mat = chrono.ChMaterialSurfaceNSC()               


radA = 1.5                                        
radB = 3.5                                        
radD = 5                                          
radE = 2                                          


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


mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis.Y,             
                                        radA, 0.5,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearA)                                                
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))                     
mbody_gearA.SetRot(chrono.Q_from_AngX(m.pi / 2))                 
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)               


mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFrame(chrono.ChVectorD(0, 3.5, 0),     
                                                          chrono.Q_from_AngX(chrono.CH_C_PI_2)))  


link_motor = chrono.ChLinkMotorRotationSpeed()                      
link_motor.Initialize(mbody_gearA, mbody_truss,                     
                      chrono.ChFrame(chrono.ChVectorD(0, 0, 0),   
                                      chrono.QUNIT))                
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))              
sys.AddLink(link_motor)                                             


interaxis12 = radA + radB                                           
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis.Y,            
                                        radB, 0.4,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearB)                                                
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -2))           
mbody_gearB.SetRot(chrono.Q_from_AngX(m.pi / 2))                 
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)               


link_revolute = chrono.ChLinkLockRevolute()                         
link_revolute.Initialize(mbody_gearB, mbody_train,                  
                         chrono.ChCoordsysD(chrono.ChVectorD(interaxis12, 0, 0), chrono.QUNIT))  
sys.AddLink(link_revolute)                                          



link_gearAB = chrono.ChLinkGear()                                     
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrame())       
link_gearAB.SetFrameA(chrono.ChFrame(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    
link_gearAB.SetFrameB(chrono.ChFrame(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    
link_gearAB.SetTau(radA / radB)                             
link_gearAB.SetPhase(0)                                         
sys.AddLink(link_gearAB)                                                  



radC = 2 * radB + radA                                                    
link_gearBC = chrono.ChLinkGear()                                     
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrame())       
link_gearBC.SetFrameA(chrono.ChFrame(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    
link_gearBC.SetFrameB(chrono.ChFrame(chrono.ChVectorD(0, 0, -4), chrono.QUNIT))        
link_gearBC.SetTau(radB / radC)                             
link_gearBC.SetInternalTeeth(True)                                            
sys.AddLink(link_gearBC)                                                  


mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis.Z,             
                                        radD, 0.7,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearD)                                                
mbody_gearD.SetPos(chrono.ChVectorD(-10, 0, -9))                     
mbody_gearD.SetRot(chrono.Q_from_AngZ(m.pi / 2))                 
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)               


link_revoluteD = chrono.ChLinkLockRevolute()                         
link_revoluteD.Initialize(mbody_gearD, mbody_truss,                  
                         chrono.ChCoordsysD(chrono.ChVectorD(-10, 0, -9), 
                                            chrono.Q_from_AngY(m.pi/2)))  
sys.AddLink(link_revoluteD)                                          


link_gearAD = chrono.ChLinkGear()                                     
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFrame())       
link_gearAD.SetFrameA(chrono.ChFrame(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    
link_gearAD.SetFrameB(chrono.ChFrame(chrono.VNULL, chrono.Q_from_AngZ(m.pi / 2)))    
link_gearAD.SetTau(1.0)                             
link_gearAD.SetBevel(True)                          
sys.AddLink(link_gearAD)                                                  


mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis.Z,             
                                         radE, 0.4,                  
                                         1000, True, False, mat)     
sys.Add(mbody_pulleyE)                                                
mbody_pulleyE.SetPos(chrono.ChVectorD(-10, -11, -9))                     
mbody_pulleyE.SetRot(chrono.Q_from_AngZ(m.pi / 2))                 
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)               


link_revoluteE = chrono.ChLinkLockRevolute()                         
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,                  
                         chrono.ChCoordsysD(chrono.ChVectorD(-10, -11, -9), 
                                            chrono.Q_from_AngY(m.pi/2)))  
sys.AddLink(link_revoluteE)                                          


link_beltDE = chrono.ChLinkPulley()                                     
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE,                  
                        chrono.ChFrame(chrono.ChVectorD(-10, 0, -9), chrono.Q_from_AngY(m.pi/2)),   
                        chrono.ChFrame(chrono.ChVectorD(-10, -11, -9), chrono.Q_from_AngY(m.pi/2)),  
                        radD, radE)                                   
sys.AddLink(link_beltDE)                                              


vis = chronoirr.ChVisualSystemIrrlicht()                        
vis.AttachSystem(sys)                                           
vis.SetWindowSize(1024, 768)                                    
vis.SetWindowTitle('Gears and pulleys')                         
vis.Initialize()                                                
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) 
vis.AddSkyBox()                                                 
vis.AddCamera(chrono.ChVectorD(12, 15, -20))                   
vis.AddTypicalLights()                                          


belt_shape = chrono.ChVisualShapeLine()
belt_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))  
belt_shape.SetThickness(0.1)  


def add_belt_visualization():
    
    belt_points = []
    
    num_points = 40
    for i in range(num_points):
        angle = 2 * m.pi * i / num_points
        
        if i < num_points / 2:
            x = -10 + radD * m.cos(angle)
            y = 0 + radD * m.sin(angle)
            belt_points.append(chrono.ChVectorD(x, y, -9))
        
        else:
            x = -10 + radE * m.cos(angle)
            y = -11 + radE * m.sin(angle)
            belt_points.append(chrono.ChVectorD(x, y, -9))
    
    
    for i in range(len(belt_points)):
        next_i = (i + 1) % len(belt_points)
        line = chrono.ChVisualShapeLine()
        line.SetLineGeometry(belt_points[i], belt_points[next_i], 0.05)  
        line.SetColor(chrono.ChColor(0.2, 0.2, 0.2))  
        mbody_truss.AddVisualShape(line)


add_belt_visualization()


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED) 


while vis.Run():                                    
    vis.BeginScene()                                
    vis.Render()                                    
    vis.EndScene()                                  
    sys.DoStepDynamics(1e-3)