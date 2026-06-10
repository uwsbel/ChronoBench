"""Epicyclic gear-and-pulley train (PyChrono, ChSystemNSC).

Models a transmission built entirely from kinematic gear/pulley constraints
(no tooth collision): a fixed truss, a rotating carrier bar, two epicycloidal
wheels A and B meshing with an internal ring on the truss (wheel C), a 1:1
bevel coupling to a side wheel D, and a synchro-belt pulley E. Wheel A is
driven at a constant angular speed by a rotation-speed motor against the fixed
truss; every other body's motion follows from the gear ratios and the belt.

System type: NSC. Bodies: truss (fixed), carrier train, gears A/B/D, pulley E.
Expected behavior: A spins at the prescribed speed, B/C/D/E follow their ratios
and the carrier orbits — smooth steady-state rotation with no divergence.
"""

import os
import math as m

import pychrono as chrono
import pychrono.irrlicht as chronoirr


# === Parameters === gear radii, truss extents, motor speed, timing constants
radA = 1.5
radB = 3.5
interaxis12 = radA + radB          # precomputed once: A-B center distance
radC = 2 * radB + radA             # precomputed once: internal ring radius
radD = 5
radE = 2
motor_speed = 3.0                  # prescribed angular speed of wheel A [rad/s]

time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # precomputed once


# === System === NSC system plus shared contact and visual materials
sys = chrono.ChSystemNSC()
mat = chrono.ChContactMaterialNSC()

vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# === Bodies === truss, rotating carrier, the epicycloidal wheels and side wheels
# ...the fixed truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# ...the rotating bar support (carrier) for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# ...the first gear A
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# ...aesthetic thin shaft cylinder on gear A (visualization only)
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape,
                           chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                                           chrono.QuatFromAngleX(chrono.CH_PI_2)))

# ...the second gear B
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# ...the bevel side gear D
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.8, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# ...the synchro-belt pulley E
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.8, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# === Joints / constraints === carrier hinge, motor on A, gear meshes, belt
# carrier rotates about Z relative to the truss at the origin
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# prescribed rotation speed between gear A and the fixed truss (full motor-link)
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(link_motor)

# gear B rides on the rotating carrier via a revolute joint
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                        chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# gear mesh A <-> B (external teeth)
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# gear mesh B <-> internal ring C (the truss acts as the fixed internal wheel)
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)     # internal-teeth ring
sys.AddLink(link_gearBC)

# side wheel D fixed to the truss with a horizontal-axis revolute
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                          chrono.ChFramed(chrono.ChVector3d(-10, 0, -9),
                                          chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteD)

# 1:1 bevel gear mesh A <-> D
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, -7, 0), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetTransmissionRatio(1)
sys.AddLink(link_gearAD)

# pulley E fixed to the truss with a horizontal-axis revolute
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                          chrono.ChFramed(chrono.ChVector3d(-10, -11, -9),
                                          chrono.QuatFromAngleY(m.pi / 2)))
sys.AddLink(link_revoluteE)

# synchro-belt constraint between wheel D and pulley E
link_pulleyDE = chrono.ChLinkLockPulley()
link_pulleyDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_pulleyDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_pulleyDE.SetRadius1(radD)
link_pulleyDE.SetRadius2(radE)
link_pulleyDE.SetEnforcePhase(True)
sys.AddLink(link_pulleyDE)

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# === Integrator === projected Euler keeps the gear constraints stable
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# === Main loop === render-cadence outer loop; batch physics between frames
gearA = mbody_gearA   # cache: protagonist body, reused every logged step

try:

    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        chronoirr.drawSegment(vis, link_pulleyDE.GetBeltUpPos1(), link_pulleyDE.GetBeltUpPos2())
        chronoirr.drawSegment(vis, link_pulleyDE.GetBeltBottomPos1(), link_pulleyDE.GetBeltBottomPos2())
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad constraint state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video + timeseries plot, then clean up
