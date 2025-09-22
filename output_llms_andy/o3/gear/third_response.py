#########################################################################
#  PYCHRONO DEMO :  compound transmission (spur gears + bevel gear      #
#                   + synchronous belt)                                 #
#                                                                       #
#  The original script has been fixed (wrong class names for frames and #
#  quaternions, wrong joint initialisations, etc.) and extended to meet #
#  the new requirements:                                                #
#     1)  Bevel gear D (radius 5) connected 1:1 to spur gear A          #
#     2)  Pulley E (radius 2) linked to gear-D by a synchronous belt    #
#     3)  Simple visual representation of the belt                      #
#########################################################################

import math as m
import pychrono as chrono
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------
# Convenience aliases that work with every recent PyChrono version
# ----------------------------------------------------------------------
def Q_from_AngX(angle):   return chrono.Q_from_AngAxis(angle, chrono.ChVector3d(1, 0, 0))
def Q_from_AngY(angle):   return chrono.Q_from_AngAxis(angle, chrono.ChVector3d(0, 1, 0))
def Q_from_AngZ(angle):   return chrono.Q_from_AngAxis(angle, chrono.ChVector3d(0, 0, 1))

# ----------------------------------------------------------------------
# Create the physical system
# ----------------------------------------------------------------------
sys = chrono.ChSystemNSC()
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# ----------------------------------------------------------------------
# Shared contact/visual materials
# ----------------------------------------------------------------------
mat_contact = chrono.ChContactMaterialNSC()

vis_mat       = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

belt_vis_mat  = chrono.ChVisualMaterial()
belt_vis_mat.SetDiffuseColor(chrono.ChColor(0.15, 0.15, 0.15))

# ----------------------------------------------------------------------
# RIGID BODIES
# ----------------------------------------------------------------------
#
#  1. Truss (fixed)
#
truss = chrono.ChBodyEasyBox(15, 8, 2,
                             1000,
                             True,  False,
                             mat_contact)
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0, 0, 3))
sys.Add(truss)

#
#  2. Rotating bar (planet carrier)
#
carrier = chrono.ChBodyEasyBox(8, 1.5, 1.0,
                               1000,
                               True, False,
                               mat_contact)
carrier.SetPos(chrono.ChVector3d(3, 0, 0))
sys.Add(carrier)

#
# Revolute joint between truss and carrier ― axis = Z
#
rev_truss_carrier = chrono.ChLinkLockRevolute()
frame_tc = chrono.ChFrameD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
rev_truss_carrier.Initialize(truss, carrier, frame_tc)
sys.AddLink(rev_truss_carrier)

#
#  3. Spur gear A
#
radA = 1.5
gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,  # cylinder main axis = Y, but we rotate later
                                  radA, 0.5,
                                  1000, True, False, mat_contact)
gearA.SetPos(chrono.ChVector3d(0, 0, -1))
gearA.SetRot(Q_from_AngX(m.pi/2))                   # move cylinder axis to global Z
gearA.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(gearA)

# A thin shaft purely for visualization
shaft_visA = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
gearA.AddVisualShape(shaft_visA,
                     chrono.ChFrameD(chrono.ChVector3d(0, 3.5, 0),
                                     Q_from_AngX(m.pi/2)))

#
# Motor: impose angular speed on gear-A w.r.t. the truss
#
motor_A = chrono.ChLinkMotorRotationSpeed()
motor_A.Initialize(gearA, truss,
                   chrono.ChFrameD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor_A.SetSpeedFunction(chrono.ChFunctionConst(3.0))   # 3 rad/s
sys.AddLink(motor_A)

#
#  4. Spur gear B  (meshing with A, mounted on carrier)
#
radB = 3.5
interaxisAB = radA + radB
gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                  radB, 0.4,
                                  1000, True, False, mat_contact)
gearB.SetPos(chrono.ChVector3d(interaxisAB, 0, -2))
gearB.SetRot(Q_from_AngX(m.pi/2))
gearB.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(gearB)

# revolute gearB ↔ carrier (axis = Z)
rev_B_carrier = chrono.ChLinkLockRevolute()
frame_B_carrier = chrono.ChFrameD(chrono.ChVector3d(interaxisAB, 0, 0), chrono.QUNIT)
rev_B_carrier.Initialize(gearB, carrier, frame_B_carrier)
sys.AddLink(rev_B_carrier)

# gear constraint A – B  (ratio = radA/radB)
gear_AB = chrono.ChLinkLockGear()
gear_AB.Initialize(gearA, gearB, chrono.ChFrameD())
gear_AB.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, Q_from_AngX(-m.pi/2)))  # Z axis
gear_AB.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, Q_from_AngX(-m.pi/2)))  # Z axis
gear_AB.SetTransmissionRatio(radA / radB)
gear_AB.SetEnforcePhase(True)
sys.AddLink(gear_AB)

#
#  5. Internal big gear C (the truss itself, radius = 2*B + A)
#
radC = 2 * radB + radA
gear_BC = chrono.ChLinkLockGear()
gear_BC.Initialize(gearB, truss, chrono.ChFrameD())
gear_BC.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, Q_from_AngX(-m.pi/2)))   # Z axis on B
gear_BC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
gear_BC.SetTransmissionRatio(radB / radC)
gear_BC.SetEpicyclic(True)
sys.AddLink(gear_BC)

# ----------------------------------------------------------------------
#  NEW PART  :  bevel gear D   +   pulley E   +   synchronous belt
# ----------------------------------------------------------------------

#
#  6.  Bevel gear D   (we approximate it with a cylinder whose axis is X)
#
radD = 5
gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,  # still built on Y but rotated to X
                                  radD, 0.6,
                                  1000, True, False, mat_contact)
gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
gearD.SetRot(Q_from_AngZ(m.pi/2))                   # bring axis to +X
gearD.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(gearD)

# Revolute gearD ↔ truss, axis = X  (z-axis of joint frame must align with X)
rev_rotX = Q_from_AngY(m.pi/2)                      # rotate frame so that local Z points to +X
rev_D_truss = chrono.ChLinkLockRevolute()
rev_D_truss.Initialize(gearD, truss,
                       chrono.ChFrameD(chrono.ChVector3d(-10, 0, -9), rev_rotX))
sys.AddLink(rev_D_truss)

# Gear constraint  (1:1)  between spur gear A (axis Z) and bevel gear D (axis X)
gear_AD = chrono.ChLinkLockGear()
gear_AD.Initialize(gearA, gearD, chrono.ChFrameD())
gear_AD.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, Q_from_AngX(-m.pi/2)))  # Z axis on A
gear_AD.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, rev_rotX))             # X axis on D
gear_AD.SetTransmissionRatio(1.0)               # 1 : 1
gear_AD.SetEnforcePhase(True)
sys.AddLink(gear_AD)

#
#  7. Pulley E  (axis = X, same rotation as gear D transmitted through a belt)
#
radE = 2
pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                    radE, 0.4,
                                    1000, True, False, mat_contact)
pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
pulleyE.SetRot(Q_from_AngZ(m.pi/2))               # axis → X
pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)
sys.Add(pulleyE)

# Revolute pulleyE ↔ truss (axis = X)
rev_E_truss = chrono.ChLinkLockRevolute()
rev_E_truss.Initialize(pulleyE, truss,
                       chrono.ChFrameD(chrono.ChVector3d(-10, -11, -9), rev_rotX))
sys.AddLink(rev_E_truss)

#
#  Belt constraint (gear-like with positive ratio but SAME spin direction)
#
belt_DE = chrono.ChLinkLockGear()
belt_DE.Initialize(gearD, pulleyE, chrono.ChFrameD())
belt_DE.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, rev_rotX))       # X axis on D
belt_DE.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, rev_rotX))       # X axis on E
belt_DE.SetTransmissionRatio(radD / radE)         # geometric belt ratio
belt_DE.SetEnforcePhase(False)                     # belts do not need phase alignment
sys.AddLink(belt_DE)

# ----------------------------------------------------------------------
#  Simple visual representation of the belt (just a thick segment)
# ----------------------------------------------------------------------
belt_segment = chrono.ChVisualShapeSegment(chrono.ChVector3d(-10, 0,  -9),
                                           chrono.ChVector3d(-10, -11, -9),
                                           0.4)
belt_segment.SetMaterial(belt_vis_mat)
truss.AddVisualShape(belt_segment)   # attach to truss (fixed body)

# ----------------------------------------------------------------------
#  VISUALISATION
# ----------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears, Bevel Gear and Synchronous Belt')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# ----------------------------------------------------------------------
#  SIMULATION LOOP
# ----------------------------------------------------------------------
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)