import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr




Vec3  = chrono.ChVectorD
Quat  = chrono.ChQuaternionD
Frame = chrono.ChFrameD
Csys  = chrono.ChCoordsysD
QUNIT = chrono.QUNIT          




radA = 1.5          
radB = 3.5          
radC = 2 * radB + radA           

density       = 1000.0           
timestep      = 1.0e-3           




sys = chrono.ChSystemNSC()


mat_surf = chrono.ChContactMaterialNSC()




truss = chrono.ChBodyEasyBox(
            15, 8, 2,            
            density,
            True,                
            False,               
            mat_surf)
truss.SetPos(Vec3(0, 0, 3))
truss.SetFixed(True)
sys.Add(truss)




vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))




carrier = chrono.ChBodyEasyBox(8, 1.5, 1.0, density, True, False, mat_surf)
carrier.SetPos(Vec3(3, 0, 0))
sys.Add(carrier)


rev_truss_carrier = chrono.ChLinkLockRevolute()
rev_truss_carrier.Initialize(truss, carrier, Csys(Vec3(0, 0, 0), QUNIT))
sys.AddLink(rev_truss_carrier)




gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5,
                                  density, True, False, mat_surf)
gearA.SetPos(Vec3(0, 0, -1))
gearA.SetRot(chrono.Q_from_AngX(math.pi / 2))
gearA.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(gearA)


shaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)   
gearA.AddVisualShape(shaft_shape,
                     Frame(Vec3(0,  3.5, 0),
                           chrono.Q_from_AngX(math.pi/2)))


motor_A = chrono.ChLinkMotorRotationSpeed()
motor_A.Initialize(gearA, truss, Csys(Vec3(0, 0, 0), QUNIT))
motor_A.SetSpeedFunction(chrono.ChFunctionConst(3.0))         
sys.AddLink(motor_A)




interaxisAB = radA + radB
gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4,
                                  density, True, False, mat_surf)
gearB.SetPos(Vec3(interaxisAB, 0, -2))        
gearB.SetRot(chrono.Q_from_AngX(math.pi / 2))
gearB.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(gearB)


rev_carrier_B = chrono.ChLinkLockRevolute()
rev_carrier_B.Initialize(gearB, carrier,
                         Csys(Vec3(interaxisAB, 0, 0), QUNIT))
sys.AddLink(rev_carrier_B)




gear_AB = chrono.ChLinkLockGear()
gear_AB.Initialize(gearA, gearB, Csys())                      
gear_AB.SetFrameShaft1(Frame(Vec3(0, 0, 0),
                             chrono.Q_from_AngX(-math.pi / 2)))
gear_AB.SetFrameShaft2(Frame(Vec3(0, 0, 0),
                             chrono.Q_from_AngX(-math.pi / 2)))
gear_AB.SetTransmissionRatio(radA / radB)   
gear_AB.SetEnforcePhase(True)
sys.AddLink(gear_AB)





gear_BC = chrono.ChLinkLockGear()
gear_BC.Initialize(gearB, truss, Csys())
gear_BC.SetFrameShaft1(Frame(Vec3(0, 0, 0),
                             chrono.Q_from_AngX(-math.pi / 2)))
gear_BC.SetFrameShaft2(Frame(Vec3(0, 0, -4), QUNIT))
gear_BC.SetTransmissionRatio(radB / radC)
gear_BC.SetEpicyclic(True)                   
sys.AddLink(gear_BC)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and Pulleys – corrected demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(Vec3(12, 15, -20))
vis.AddTypicalLights()




sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)