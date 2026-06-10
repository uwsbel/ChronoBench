"""Gear train with bevel gear, pulley and synchro belt (PyChrono 9.0.x, NSC).

Models a kinematic transmission built from rigid bodies coupled only by
constraint links (no contact/collision):

  * Gear A : a motor-driven spur gear that spins about the world Y axis at a
             constant angular speed, hinged to a fixed truss by a revolute.
  * Gear B : an idler/output spur gear meshing with gear A (revolute to truss),
             coupled to A by a ChLinkLockGear so their pitch circles roll.
  * Gear D : a bevel gear (radius 5) located at (-10, 0, -9), rotated 90 deg
             about Z, hinged to the truss by a revolute along a horizontal axis,
             and coupled to gear A by a 1:1 ChLinkLockGear (bevel transmission).
  * Pulley E : a pulley (radius 2) located at (-10, -11, -9), rotated 90 deg
             about Z, hinged to the truss by a revolute along a horizontal axis,
             and coupled to gear D by a ChLinkLockPulley synchro-belt constraint.

The transmission is purely kinematic (gear/pulley couplings, not surface
contact), so no collision system is configured. Expected behavior: A drives B
and D through the gear constraints; D drives E through the belt; every wheel
rotates smoothly at the speed dictated by the ratios, with no divergence.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 2.0e-3          # integration step [s]
sim_end = 8.0               # simulation duration [s]
render_fps = 50.0           # review render cadence [frames/s]

motor_speed = 0.5 * math.pi  # gear A drive speed [rad/s]

radA = 6.0                  # spur gear A pitch radius [m]
radB = 4.0                  # spur gear B pitch radius [m]
radD = 5.0                  # bevel gear D pitch radius [m]
radE = 2.0                  # pulley E pitch radius [m]
gear_thick = 1.0            # gear/pulley axial thickness [m]
density = 1000.0            # body material density [kg/m^3]

# Positions (Z-up world). Gear A is the drive at the origin region.
posA = chrono.ChVector3d(0, 0, 0)
posB = chrono.ChVector3d(radA + radB, 0, 0)   # B meshes with A along +X
posD = chrono.ChVector3d(-10, 0, -9)          # bevel gear D
posE = chrono.ChVector3d(-10, -11, -9)        # pulley E

# Rotations: gears A/B spin about world Y; D and E are rotated 90 deg about Z.
rot_Y_spin = chrono.QuatFromAngleX(chrono.CH_PI_2)          # cyl Y-axis stays Y
rot_Zaxis_90 = chrono.QuatFromAngleZ(chrono.CH_PI_2)        # 90 deg about Z

# === System & gravity === single NSC system; pure jointed transmission
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Smooth, stable constraint stepping for the coupled gear/belt links.
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# === Bodies === fixed truss plus four coupled wheels (cylinders along their spin axis)
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
sys.AddBody(truss)
truss_post = chrono.ChVisualShapeBox(0.6, 0.6, 20.0)
truss_post.SetColor(chrono.ChColor(0.3, 0.3, 0.35))
truss.AddVisualShape(truss_post, chrono.ChFramed(chrono.ChVector3d(-10, -5.5, -9), chrono.QUNIT))

# Gear A — spur gear, spin axis = world Y (cylinder local Y along world Y).
gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, gear_thick, density, True, False)
gearA.SetPos(posA)
gearA.SetName("gearA")
gearA.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
# Textured so the axial spin is visually apparent (a plain disc looks static).
gearA.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/checker2.png"))
sys.AddBody(gearA)

# Gear B — output spur gear meshing with A, spin axis = world Y.
gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, gear_thick, density, True, False)
gearB.SetPos(posB)
gearB.SetName("gearB")
gearB.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.8, 0.2))
gearB.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/checker2.png"))
sys.AddBody(gearB)

# Gear D — bevel gear, rotated 90 deg about Z so its spin axis lies horizontal (world X).
gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, gear_thick, density, True, False)
gearD.SetPos(posD)
gearD.SetRot(rot_Zaxis_90)
gearD.SetName("gearD")
gearD.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.4, 0.9))
gearD.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/checker2.png"))
sys.AddBody(gearD)

# Pulley E — rotated 90 deg about Z so its spin axis is horizontal (world X).
pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, gear_thick, density, True, False)
pulleyE.SetPos(posE)
pulleyE.SetRot(rot_Zaxis_90)
pulleyE.SetName("pulleyE")
pulleyE.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.7, 0.1))
pulleyE.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/checker2.png"))
sys.AddBody(pulleyE)

# === Joints / constraints === revolutes to the truss, gear meshes, belt, drive motor
# Revolute hinge axis = each wheel's spin axis. ChLinkLockRevolute uses frame local +Z
# as the hinge, so map +Z onto the physical spin axis with the right quaternion.
q_hingeY = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)  # local +Z -> world Y
q_hingeX = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_Y)  # local +Z -> world X

revA = chrono.ChLinkLockRevolute()
revA.Initialize(gearA, truss, chrono.ChFramed(posA, q_hingeY))
sys.AddLink(revA)

revB = chrono.ChLinkLockRevolute()
revB.Initialize(gearB, truss, chrono.ChFramed(posB, q_hingeY))
sys.AddLink(revB)

revD = chrono.ChLinkLockRevolute()
revD.Initialize(gearD, truss, chrono.ChFramed(posD, q_hingeX))
sys.AddLink(revD)

revE = chrono.ChLinkLockRevolute()
revE.Initialize(pulleyE, truss, chrono.ChFramed(posE, q_hingeX))
sys.AddLink(revE)

# Drive motor: gear A spins at a constant angular speed about its spin axis (world Y).
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gearA, truss, chrono.ChFramed(posA, q_hingeY))
motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(motor)

# Spur mesh A<->B. Shaft frames are body-LOCAL frames whose Z is the spin axis.
# gearA/gearB local +Z must point along world Y (their spin axis): rotate X by 90 deg.
shaftframe_A = chrono.ChFramed(chrono.VNULL, q_hingeY)
shaftframe_B = chrono.ChFramed(chrono.VNULL, q_hingeY)

gearAB = chrono.ChLinkLockGear()
gearAB.Initialize(gearA, gearB, chrono.ChFramed())
gearAB.SetFrameShaft1(shaftframe_A)
gearAB.SetFrameShaft2(shaftframe_B)
# Transmission ratio from the pitch radii (z1:z2 proportional to radii).
gearAB.SetTransmissionRatio(radA, radB)
gearAB.SetEnforcePhase(True)
sys.AddLink(gearAB)

# Bevel transmission A<->D at 1:1. gearD is rotated 90 deg about Z, so in its OWN
# local frame the spin axis (world X) is the body-local Y; map local +Z onto local Y.
shaftframe_D = chrono.ChFramed(chrono.VNULL, q_hingeY)

gearAD = chrono.ChLinkLockGear()
gearAD.Initialize(gearA, gearD, chrono.ChFramed())
gearAD.SetFrameShaft1(shaftframe_A)
gearAD.SetFrameShaft2(shaftframe_D)
gearAD.SetTransmissionRatio(1.0)   # 1:1 gear ratio between gear A and gear D
gearAD.SetEpicyclic(False)
sys.AddLink(gearAD)

# Synchro belt D<->E (ChLinkLockPulley). Same body-local shaft-frame convention.
shaftframe_E = chrono.ChFramed(chrono.VNULL, q_hingeY)

beltDE = chrono.ChLinkLockPulley()
beltDE.Initialize(gearD, pulleyE, chrono.ChFramed())
beltDE.SetFrameShaft1(shaftframe_D)
beltDE.SetFrameShaft2(shaftframe_E)
beltDE.SetRadius1(radD)
beltDE.SetRadius2(radE)
sys.AddLink(beltDE)

# Simplified belt visual: a line on the truss spanning gear D and pulley E centers.
belt_vis = chrono.ChVisualShapeLine()
belt_vis.SetLineGeometry(chrono.ChLineSegment(posD, posE))
belt_vis.SetColor(chrono.ChColor(0.05, 0.05, 0.05))
truss.AddVisualShape(belt_vis)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear train with bevel gear, pulley and belt")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(14, -20, 12), chrono.ChVector3d(-6, -4, -5))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(-5, -5, -12), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === real-time render-cadence loop; physics batched between frames
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once
get_time = sys.GetChTime                                       # cache: getter reused every step


frame = 0
try:
    while vis.Run() and get_time() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if get_time() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video + plot, then clean frames
