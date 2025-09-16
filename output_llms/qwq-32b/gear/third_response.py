import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

sys = chrono.ChSystemNSC()
mat = chrono.ChContactMaterialNSC()
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create truss
radA = 1.5
radB = 3.5
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVectorD(0, 0, 3))

# Rotating bar (train)
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVectorD(3, 0, 0))

# Revolute joint between truss and train
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train, 
                          chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# First gear (A)
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi/2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Motor for gear A
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, 
                      chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))
sys.AddLink(link_motor)

# Second gear (B)
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi/2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Joint between gear B and train
link_revoluteB = chrono.ChLinkLockRevolute()
link_revoluteB.Initialize(mbody_gearB, mbody_train, 
                         chrono.ChFrameD(chrono.ChVectorD(interaxis12,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteB)

# Gear constraint between A and B
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD())
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QuatFromAngleX(-m.pi/2)))
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QuatFromAngleX(-m.pi/2)))
link_gearAB.SetTransmissionRatio(radA/radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Gear constraint between B and truss (C)
radC = 2*radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD())
link_gearBC.SetFrameShaft1(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QuatFromAngleX(-m.pi/2)))
link_gearBC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0,0,-4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB/radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# Bevel gear D
radD = 5.0
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 1.0, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVectorD(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi/2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for gear D (axis along X)
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_truss, mbody_gearD, 
                         chrono.ChFrameD(chrono.ChVectorD(-10,0,-9), 
                                        chrono.QuatFromAngleY(-m.pi/2)))
sys.AddLink(link_revoluteD)

# Gear constraint between A and D (1:1 ratio)
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFrameD())
link_gearAD.SetFrameShaft1(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QuatFromAngleX(-m.pi/2)))
link_gearAD.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QuatFromAngleY(m.pi/2)))
link_gearAD.SetTransmissionRatio(1.0)
sys.AddLink(link_gearAD)

# Pulley E
radE = 2.0
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.5, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVectorD(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi/2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for pulley E (axis along Y)
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_truss, mbody_pulleyE, 
                         chrono.ChFrameD(chrono.ChVectorD(-10,-11,-9), 
                                        chrono.QuatFromAngleX(m.pi/2)))
sys.AddLink(link_revoluteE)

# Belt constraint between D and E
link_beltDE = chrono.ChLinkBeltSynchro()
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, 
                      chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT),
                      chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
sys.AddLink(link_beltDE)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Advanced Gear System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(12, 15, -20))
vis.AddTypicalLights()
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop with belt visualization
while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    # Draw belt between D and E
    posD = mbody_gearD.GetPos()
    posE = mbody_pulleyE.GetPos()
    vis.GetSystem().GetVisualSystem().GetRenderScene().AddSegment(posD, posE, chrono.ChColor(0.5,0.5,0.5))
    
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Initialize system
sys = chrono.ChSystemNSC()
mat = chrono.ChContactMaterialNSC()
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create truss
radA = 1.5
radB = 3.5
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVectorD(0, 0, 3))

# Rotating bar (train)
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVectorD(3, 0, 0))

# Revolute joint between truss and train
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train, 
                          chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# First gear (A)
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi/2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Motor for gear A
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, 
                      chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))
sys.AddLink(link_motor)

# Second gear (B)
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi/2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Joint between gear B and train
link_revoluteB = chrono.ChLinkLockRevolute()
link_revoluteB.Initialize(mbody_gearB, mbody_train, 
                         chrono.ChFrameD(chrono.ChVectorD(interaxis12,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteB)

# Gear constraint between A and B
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD())
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QuatFromAngleX(-m.pi/2)))
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QuatFromAngleX(-m.pi/2)))
link_gearAB.SetTransmissionRatio(radA/radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Gear constraint between B and truss (C)
radC = 2*radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD())
link_gearBC.SetFrameShaft1(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QuatFromAngleX(-m.pi/2)))
link_gearBC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0,0,-4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB/radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# Bevel gear D
radD = 5.0
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 1.0, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVectorD(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi/2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for gear D (axis along X)
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_truss, mbody_gearD, 
                         chrono.ChFrameD(chrono.ChVectorD(-10,0,-9), 
                                        chrono.QuatFromAngleY(-m.pi/2)))
sys.AddLink(link_revoluteD)

# Gear constraint between A and D (1:1 ratio)
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFrameD())
link_gearAD.SetFrameShaft1(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QuatFromAngleX(-m.pi/2)))
link_gearAD.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QuatFromAngleY(m.pi/2)))
link_gearAD.SetTransmissionRatio(1.0)
sys.AddLink(link_gearAD)

# Pulley E
radE = 2.0
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.5, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVectorD(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi/2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for pulley E (axis along Y)
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_truss, mbody_pulleyE, 
                         chrono.ChFrameD(chrono.ChVectorD(-10,-11,-9), 
                                        chrono.QuatFromAngleX(m.pi/2)))
sys.AddLink(link_revoluteE)

# Belt constraint between D and E
link_beltDE = chrono.ChLinkBeltSynchro()
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, 
                      chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT),
                      chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
sys.AddLink(link_beltDE)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Advanced Gear System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(12, 15, -20))
vis.AddTypicalLights()
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop with belt visualization
while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    # Draw belt between D and E
    posD = mbody_gearD.GetPos()
    posE = mbody_pulleyE.GetPos()
    vis.GetSystem().GetVisualSystem().GetRenderScene().AddSegment(posD, posE, chrono.ChColor(0.5,0.5,0.5))
    
    vis.EndScene()
    sys.DoStepDynamics(1e-3)