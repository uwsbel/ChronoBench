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
                           chrono.ChFrameD(chrono.ChVectorD(0,0,0),     
                                           chrono.QUNIT))             
sys.AddLink(link_revoluteTT)                                          


mbody_gearA = chrono.ChBodyEasyCylinder(                              
    radA, 0.5,                                                        
    1000, True, False, mat)                                           

sys.Add(mbody_gearA)                                                  
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))                       
mbody_gearA.SetRot(chrono.Q_from_AngX(m.pi / 2))                     
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)                


mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0),     
                                                          chrono.Q_from_AngX(chrono.CH_C_PI_2)))  


link_motor = chrono.ChLinkMotorRotationSpeed()                      
link_motor.Initialize(mbody_gearA, mbody_truss,                     
                      chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),     
                                      chrono.QUNIT))                
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))             
sys.AddLink(link_motor)                                             


interaxis12 = radA + radB                                           
mbody_gearB = chrono.ChBodyEasyCylinder(
    radB, 0.4,                                                      
    1000, True, False, mat)                                         
sys.Add(mbody_gearB)                                                
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -2))           
mbody_gearB.SetRot(chrono.Q_from_AngX(m.pi / 2))                   
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)              


link_revolute = chrono.ChLinkLockRevolute()                         
link_revolute.Initialize(mbody_gearB, mbody_train,                  
                         chrono.ChFrameD(chrono.ChVectorD(interaxis12, 0, 0), chrono.QUNIT))  
sys.AddLink(link_revolute)                                          



link_gearAB = chrono.ChLinkLockGear()                                     
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD())       
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    
link_gearAB.SetTransmissionRatio(radA / radB)                             
link_gearAB.SetEnforcePhase(True)                                         
sys.AddLink(link_gearAB)                                                  



radC = 2 * radB + radA                                                    
link_gearBC = chrono.ChLinkLockGear()                                     
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD())       
link_gearBC.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    
link_gearBC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0, 0, -4), chrono.QUNIT))        
link_gearBC.SetTransmissionRatio(radB / radC)                             
link_gearBC.SetEpicyclic(True)                                            
sys.AddLink(link_gearBC)                                                  




radD = 5
mbody_gearD = chrono.ChBodyEasyCylinder(
    radD, 0.5,                                                      
    1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVectorD(-10, 0, -9))
mbody_gearD.SetRot(chrono.Q_from_AngZ(m.pi / 2))                    
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)


mshaft_shape_D = chrono.ChVisualShapeCylinder(radD * 0.3, 10)
mbody_gearD.AddVisualShape(mshaft_shape_D, chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0),
                                                           chrono.Q_from_AngX(chrono.CH_C_PI_2)))


link_revolute_D_truss = chrono.ChLinkLockRevolute()


link_revolute_D_truss.Initialize(
    mbody_truss, mbody_gearD,
    chrono.ChFrameD(chrono.ChVectorD(-10, 0, -9), chrono.QUNIT))
sys.AddLink(link_revolute_D_truss)


link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFrameD())




link_gearAD.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))






link_gearAD.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.QUNIT))

link_gearAD.SetTransmissionRatio(1.0)  
link_gearAD.SetEnforcePhase(True)
sys.AddLink(link_gearAD)


radE = 2
mbody_pulleyE = chrono.ChBodyEasyCylinder(
    radE, 0.4,                              
    1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVectorD(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.Q_from_AngZ(m.pi / 2))                
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)


mshaft_shape_E = chrono.ChVisualShapeCylinder(radE * 0.3, 10)
mbody_pulleyE.AddVisualShape(mshaft_shape_E, chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0),
                                                           chrono.Q_from_AngX(chrono.CH_C_PI_2)))


link_revolute_E_truss = chrono.ChLinkLockRevolute()
link_revolute_E_truss.Initialize(
    mbody_truss, mbody_pulleyE,
    chrono.ChFrameD(chrono.ChVectorD(-10, -11, -9), chrono.QUNIT))
sys.AddLink(link_revolute_E_truss)





link_belt_DE = chrono.ChLinkGear()
link_belt_DE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFrameD())



link_belt_DE.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.QUNIT))
link_belt_DE.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.QUNIT))



link_belt_DE.SetTransmissionRatio(radD / radE)
link_belt_DE.SetEnforcePhase(True)
sys.AddLink(link_belt_DE)





class BeltVisual(chronoirr.ChIrrUpgrade):
    def __init__(self, vis_sys, body1, body2):
        super().__init__()
        self.vis_sys = vis_sys
        self.body1 = body1
        self.body2 = body2
        
        from pychrono import irrlicht as irr
        driver = self.vis_sys.GetDevice().getVideoDriver()
        smgr = self.vis_sys.GetDevice().getSceneManager()
        self.line_node = smgr.addAnimatedMeshSceneNode(irr.createLine([irr.vector3df(0,0,0), irr.vector3df(1,1,1)]))
        self.line_node.setMaterialFlag(irr.EMF_LIGHTING, False)
        self.line_node.setMaterialType(irr.EMT_TRANSPARENT_ADD_COLOR)
        self.line_node.setMaterialTexture(0, None)
        self.line_node.getMaterial(0).EmissiveColor = irr.SColor(255, 255, 255, 100)  
        self.line_node.setVisible(True)

    def Update(self):
        from pychrono import irrlicht as irr
        if not self.vis_sys.Run():
            return
        pos1 = self.body1.GetPos()
        pos2 = self.body2.GetPos()
        
        
        dir_vec = pos2 - pos1
        
        v0 = irr.vector3df(float(pos1.x), float(pos1.y), float(pos1.z))
        v1 = irr.vector3df(float(pos2.x), float(pos2.y), float(pos2.z))
        
        
        
        
        node = self.line_node
        smgr = self.vis_sys.GetDevice().getSceneManager()
        smgr.getRootSceneNode().removeChild(node)
        self.line_node.remove()
        
        self.line_node = smgr.addAnimatedMeshSceneNode(irr.createLine([v0, v1]))
        self.line_node.setMaterialFlag(irr.EMF_LIGHTING, False)
        self.line_node.setMaterialType(irr.EMT_TRANSPARENT_ADD_COLOR)
        self.line_node.setMaterialTexture(0, None)
        self.line_node.getMaterial(0).EmissiveColor = irr.SColor(255, 255, 255, 150)
        self.line_node.setVisible(True)






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
sys.SetTimestep(1e-3)                                           


def draw_belt(vis_sys, pos1, rad1, pos2, rad2):
    
    
    
    smgr = vis_sys.GetDevice().getSceneManager()
    driver = vis_sys.GetDevice().getVideoDriver()

    vec_start = chrono.ChVectorD(pos1.x, pos1.y, pos1.z)
    vec_end = chrono.ChVectorD(pos2.x, pos2.y, pos2.z)

    
    from pychrono import irrlicht as irr
    irr_start = irr.vector3df(float(pos1.x), float(pos1.y), float(pos1.z))
    irr_end = irr.vector3df(float(pos2.x), float(pos2.y), float(pos2.z))

    
    driver.setTransform(irr.ETS_WORLD, irr.matrix4())
    driver.draw3DLine(irr_start, irr_end, irr.SColor(255, 100, 100, 255))

    
    
    smgr.addSphereSceneNode(rad1*0.05, 12, None, -1, irr_start, irr.vector3df(1,1,1), irr.SColor(150,255,255,255))
    smgr.addSphereSceneNode(rad2*0.05, 12, None, -1, irr_end, irr.vector3df(1,1,1), irr.SColor(150,255,255,255))



while vis.Run():                                    
    vis.BeginScene()                                
    vis.Render()                                    

    
    draw_belt(vis, mbody_gearD.GetPos(), radD, mbody_pulleyE.GetPos(), radE)

    vis.EndScene()                                  
    sys.DoStepDynamics(1e-3)