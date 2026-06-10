"""Epicyclic gear train with a bevel gear and a synchro-belt pulley (PyChrono, NSC).

Models a motor-driven planetary gear train plus a side branch:
  - truss        : fixed reference box (also acts as the internal ring gear C)
  - train bar    : rotating carrier hinged to the truss along Z
  - gear A       : motor-driven first gear, spun by a constant-speed motor vs. truss
  - gear B       : planet gear carried by the train bar, meshing A (ChLinkLockGear)
  - gear C       : the truss itself, internal/epicyclic mesh with B
  - gear D       : bevel gear on a horizontal shaft, 1:1 gear constraint to gear A
  - pulley E     : pulley on a horizontal shaft, driven from gear D by a synchro belt

Constraints: ChLinkLockGear (A-B, B-C epicyclic, A-D 1:1) and ChLinkLockPulley (D-E).
This is a pure jointed multibody system (no contact), so no collision system is set.
Expected behavior: A spins steadily; B/D/E rotate at ratios fixed by the gear/belt
constraints and the carrier bar revolves, all running smoothly with no divergence.
"""

import math as m

import pychrono as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === geometry, radii, motor speed and timing (named once)
radA = 2.0          # first (driving) gear radius
radB = 4.0          # planet gear radius
radC = 2 * radB + radA   # internal ring (truss) effective radius
radD = 5.0          # bevel gear radius
radE = 2.0          # pulley radius
motor_speed = 6.0   # rad/s prescribed on gear A
interaxis12 = radA + radB    # A-B centre distance
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# === System & gravity === single NSC system; pure jointed MBS (no collision)
sys = chrono.ChSystemNSC()
mat = chrono.ChContactMaterialNSC()   # shared visual/contact material handle

# === Bodies === truss, carrier bar, gears A/B/D and pulley E (cylinders, Y axis)
mbody_truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# rotating carrier bar that supports the planet gear
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# first (driving) gear A
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)
# thin shaft cylinder, purely for visualization of gear A's axle
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.4, 13)
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))

# planet gear B
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -1))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# bevel gear D on a horizontal shaft, rotated 90 deg about Z
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.8, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# pulley E on a horizontal shaft, rotated 90 deg about Z
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.8, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# === Joints / constraints === revolutes, the speed motor, gear meshes and belt
# carrier bar rotates relative to truss about Z at the origin
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# constant-speed motor between gear A and the fixed truss (full motor-link)
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(link_motor)

# planet gear B hinged to the rotating carrier bar
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# gear mesh A-B; shaft frames rotated so local +Z is the wheel (Y) axis
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# epicyclic mesh between planet B and the internal ring (truss as gear C)
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)   # internal-teeth (ring) mesh
sys.AddLink(link_gearBC)

# bevel gear D fixed to the truss with a horizontal-axis revolute
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                          chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteD)

# 1:1 bevel gear constraint between gear A and gear D
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetTransmissionRatio(1)
sys.AddLink(link_gearAD)

# pulley E fixed to the truss with a horizontal-axis revolute
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                          chrono.ChFramed(chrono.ChVector3d(-10, -11, -9), chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteE)

# synchro-belt constraint coupling bevel gear D and pulley E
link_pulleyDE = chrono.ChLinkLockPulley()
link_pulleyDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_pulleyDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetRadius1(radD)
link_pulleyDE.SetRadius2(radE)
link_pulleyDE.SetEnforcePhase(True)   # synchro belts must not slip
sys.AddLink(link_pulleyDE)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# === Integrator === projected Euler keeps the gear/belt constraints stable
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# === Main loop === render-cadence outer loop; physics batch advances between frames

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        # simplified belt: line segments between the two pulley tangent points
        chronoirr.drawSegment(vis, link_pulleyDE.GetBeltUpPos1(), link_pulleyDE.GetBeltUpPos2())
        chronoirr.drawSegment(vis, link_pulleyDE.GetBeltBottomPos1(), link_pulleyDE.GetBeltBottomPos2())
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
