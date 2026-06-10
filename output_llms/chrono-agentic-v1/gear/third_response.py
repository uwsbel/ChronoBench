"""
Gear and Pulley Mechanism — PyChrono MBS Simulation (ChSystemNSC)

Models an epicycloidal gear train with an extended bevel-gear + synchro-belt subsystem:
  - Truss (fixed ground body)
  - Gear A (drive gear, r=2): motor-driven at 6 rad/s via ChLinkMotorRotationSpeed
  - Rotating bar (train): revolute to truss, carries gear B
  - Gear B (planet gear, r=4): meshes with A (gear ratio) and with ring C (epicyclic)
  - Ring C (r=10): internal ring gear = truss body itself
  - Gear D (bevel gear, r=5): at (-10, 0, -9), rotated 90 degrees about Z, revolute to
                               truss on horizontal axis; 1:1 gear ratio with gear A
  - Pulley E (r=2): at (-10, -11, -9), rotated 90 degrees about Z, revolute to truss on
                    horizontal axis; synchro-belt constraint (ChLinkLockPulley) with gear D
  - Belt visualization: two line segments (upper and lower strand) drawn between D and E
Expected behavior: motor drives gear A; gear D rotates 1:1 with A via bevel gear link;
pulley E is driven by synchro belt from D; epicycloidal train revolves.
"""

import math as m
import os


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
RAD_A = 2       # drive gear radius [m]
RAD_B = 4       # planet gear radius [m]
RAD_D = 5       # bevel gear radius [m]
RAD_E = 2       # pulley radius [m]

TIME_STEP  = 1e-3   # physics step size [s]
SIM_END    = 12.0   # simulation end time [s]

# Derived constants — precomputed once
INTERAXIS_AB = RAD_A + RAD_B        # center distance A-B
RAD_C        = 2 * RAD_B + RAD_A   # ring gear radius (epicyclic)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Contact material (NSC; no collision shapes for pure gear MBS)
mat = chrono.ChContactMaterialNSC()

# Shared visual material — pink-white texture for all gears
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# === Bodies ===

# --- Truss (fixed ground box, also serves as ring gear C) ---
mbody_truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# --- Rotating carrier bar (holds planet gear B, revolves around gear A axis) ---
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Train revolute to truss about Z at origin
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                            chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# --- Gear A (sun/drive gear, r=2, axis Y, rotated 90 deg about X to align Y as world-X) ---
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, RAD_A, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Decorative shaft visual on gear A
mshaft_shape = chrono.ChVisualShapeCylinder(RAD_A * 0.4, 13)
mbody_gearA.AddVisualShape(mshaft_shape,
                            chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                                            chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Motor: gear A vs truss — full motor-link, no companion revolute needed
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                       chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(6))
sys.AddLink(link_motor)

# --- Gear B (planet gear, r=4, carried by rotating train) ---
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, RAD_B, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(INTERAXIS_AB, 0, -1))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Gear B revolute to rotating carrier train
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChFramed(chrono.ChVector3d(INTERAXIS_AB, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# Gear constraint A-B (external mesh, ratio = radA / radB)
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(RAD_A / RAD_B)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Gear constraint B-C: planet B meshes with inner ring C (truss), epicyclic
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(RAD_B / RAD_C)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# --- Gear D (bevel gear, r=5, axis Y rotated 90 deg about Z for horizontal axis) ---
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, RAD_D, 0.8, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# Gear D revolute to truss — horizontal axis (QuatFromAngleY(pi/2) aligns local Z to world X)
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                           chrono.ChFramed(chrono.ChVector3d(-10, 0, -9),
                                           chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteD)

# Bevel gear A-D: 1:1 transmission ratio
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0, -7, 0),
                                            chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, -7, 0),
                                            chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetTransmissionRatio(1)
sys.AddLink(link_gearAD)

# --- Pulley E (synchro-belt follower, r=2, same axis orientation as D) ---
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, RAD_E, 0.8, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# Pulley E revolute to truss — same horizontal axis convention as gear D
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                           chrono.ChFramed(chrono.ChVector3d(-10, -11, -9),
                                           chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteE)

# Synchro belt constraint D-E: ChLinkLockPulley enforces equal belt-linear-velocity
link_pulleyDE = chrono.ChLinkLockPulley()
link_pulleyDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_pulleyDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetRadius1(RAD_D)
link_pulleyDE.SetRadius2(RAD_E)
link_pulleyDE.SetEnforcePhase(True)
sys.AddLink(link_pulleyDE)

# Timestepper (implicit projected Euler matches canonical gear demo)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gear Train with Bevel Gear D and Synchro Belt Pulley E')
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -6, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop ===


frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        # Belt visualization: draw upper and lower belt strands between D and E
        chronoirr.drawSegment(vis,
                              link_pulleyDE.GetBeltUpPos1(),
                              link_pulleyDE.GetBeltUpPos2())
        chronoirr.drawSegment(vis,
                              link_pulleyDE.GetBeltBottomPos1(),
                              link_pulleyDE.GetBeltBottomPos2())
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
