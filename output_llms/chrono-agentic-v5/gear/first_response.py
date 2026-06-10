"""Epicyclic gear train simulation (PyChrono 9.0.0, NSC, Irrlicht).

Models a planetary/epicyclic reducer built from kinematic gear constraints
(no tooth-collision geometry):
  * a fixed truss (the stationary support / outer ring with internal teeth),
  * a rotating bar (the carrier / train arm) hinged to the truss on the Z axis,
  * gear A (the sun) driven at a constant angular speed by a rotation-speed motor,
  * gear B (the planet) carried on the rotating bar.
The two gears mesh via ChLinkLockGear, and gear B also meshes against the fixed
truss as an internal (epicyclic) ring, so driving the sun at constant speed makes
the planet roll and the carrier bar revolve. Expected behavior: gear A spins at the
prescribed constant rate, gear B and the carrier bar rotate steadily with no
free-fall (all rotations are kinematically constrained — no contact/collision).
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / motor / timing (named constants, no bare literals)
radA = 2.0                 # sun gear radius
radB = 4.0                 # planet gear radius
interaxis = radA + radB    # center distance sun -> planet
radC = 2.0 * radB + radA   # internal ring (truss) pitch radius
motor_speed = 6.0          # prescribed sun angular speed [rad/s]
gear_z = -1.0              # gear plane offset along Z
time_step = 1e-3
sim_end = 12.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once


# === System & gravity === pure jointed MBS (gear/revolute constraints, no contact)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Shared contact material (carried by the ChBodyEasy* bodies; no collision is used)
mat = chrono.ChContactMaterialNSC()

# === Bodies === truss (fixed), carrier bar, sun gear A, planet gear B
# ...the truss: fixed support, also acts as the internal-tooth ring wheel C
truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False, mat)
sys.Add(truss)
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0, 0, 3))

# ...the rotating bar that supports the planet (the carrier / train arm)
train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(train)
train.SetPos(chrono.ChVector3d(interaxis * 0.5, 0, 0))

# ...the first (sun) gear A, cylinder with its axis along Y, laid into the XY plane
gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(gearA)
gearA.SetPos(chrono.ChVector3d(0, 0, gear_z))
gearA.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))

# ...the second (planet) gear B
gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(gearB)
gearB.SetPos(chrono.ChVector3d(interaxis, 0, gear_z))
gearB.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))

# === Joints / constraints === carrier hinge, motor, planet hinge, two gear meshes
# carrier bar rotates about the truss on the Z axis through the origin
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(truss, train, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# rotation-speed motor between sun gear A and the fixed truss (full motor-link:
# it imposes the revolute itself, so NO separate revolute at this pivot)
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(gearA, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(link_motor)

# planet gear B is hinged to the rotating carrier bar
link_revoluteB = chrono.ChLinkLockRevolute()
link_revoluteB.Initialize(gearB, train, chrono.ChFramed(chrono.ChVector3d(interaxis, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteB)

# gear mesh sun(A) <-> planet(B): shaft axes are local Z, so rotate frames -90deg
# about X because ChBodyEasyCylinder builds the cylinder with its axis along Y
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(gearA, gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# gear mesh planet(B) <-> internal ring on the fixed truss (epicyclic / inner teeth)
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(gearB, truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)   # internal-tooth ring wheel
sys.AddLink(link_gearBC)

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Epicyclic gear train")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20), chrono.ChVector3d(3, 0, -1))
vis.AddTypicalLights()


# === Main loop === advance the gear train, render at a fixed frame cadence
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
