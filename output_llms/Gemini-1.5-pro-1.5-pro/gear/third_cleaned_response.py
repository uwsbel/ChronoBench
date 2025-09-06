import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m


sys = chrono.ChSystemNSC()


mat = chrono.ChContactMaterialNSC()


radA = 1.5
radB = 3.5
radD = 5  
radE = 2  


mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))


vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))


mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))


link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0))) 
sys.AddLink(link_revoluteTT)



mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.Q_from_AngX(m.pi / 2))  
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFrameD(chrono.ChVector3d(0, 3.5, 0), chrono.Q_from_AngX(chrono.CH_PI_2)))



link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0))) 
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))  
sys.AddLink(link_motor)


interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.Q_from_AngX(m.pi / 2)) 
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)


link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train, chrono.ChCoordsysD(chrono.ChVector3d(interaxis12, 0, 0))) 
sys.AddLink(link_revolute)



mbody_gearD = chrono.ChBodyEasyBevelGear(radD, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.Q_from_AngZ(m.pi / 2)) 
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)


link_revolute_D = chrono.ChLinkLockRevolute()
link_revolute_D.Initialize(mbody_gearD, mbody_truss, chrono.ChCoordsysD(chrono.ChVector3d(-10, 0,-9), chrono.Q_from_AngAxis(chrono.CH_PI/2, chrono.VECT_Y)))
sys.AddLink(link_revolute_D)



link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChCoordsysD())
link_gearAD.SetTransmissionRatio(1)  

sys.AddLink(link_gearAD)





mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.3, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.Q_from_AngZ(m.pi / 2))  
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)



link_revolute_E = chrono.ChLinkLockRevolute()
link_revolute_E.Initialize(mbody_pulleyE, mbody_truss,  chrono.ChCoordsysD(chrono.ChVector3d(-10, -11, -9), chrono.Q_from_AngAxis(chrono.CH_PI/2, chrono.VECT_Y)))
sys.AddLink(link_revolute_E)




link_beltDE = chrono.ChLinkBeltSynchro()
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChCoordsysD(), chrono.CoordsysD(), radD, radE) 
sys.AddLink(link_beltDE)









belt_asset = chrono.ChVisualShapeLine()
belt_asset.SetPoints(chrono.ChVector3d(), chrono.ChVector3d()) 
belt_asset.SetColor(chrono.ChColor(0, 0, 1))  
belt_asset.SetThickness(0.2)
sys.AddVisualShape(belt_asset) 


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    
    posD = mbody_gearD.GetPos()
    posE = mbody_pulleyE.GetPos()
    belt_asset.SetPoints(chrono.ChVector3d(posD.x, posD.y, posD.z-radD*0.5) , chrono.ChVector3d(posE.x, posE.y, posE.z-radE*0.5))

    sys.DoStepDynamics(1e-3)