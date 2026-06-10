"""Spur-gear pair driven by a rotational-speed motor (PyChrono 9.0.1, Irrlicht).

Models a planar two-gear transmission as a pure jointed multi-body system
(ChSystemNSC, no contact/collision — the meshing is enforced kinematically by
ChLinkLockGear, not by tooth contact):

  * truss       : a large fixed box acting as the stationary support/frame.
  * gear A      : the small driving gear (radius radA), spinning about its shaft.
  * gear B      : the larger driven gear (radius radB), on a parallel shaft.

Topology:
  * gear A  -- ChLinkMotorRotationSpeed --> truss : imposes a constant spin.
  * gear A  -- ChLinkLockRevolute        --> truss : the A shaft bearing.
  * gear B  -- ChLinkLockRevolute        --> truss : the B shaft bearing.
  * gear A  -- ChLinkLockGear            --> gear B : the kinematic mesh
              (transmission ratio radA/radB; shaft axes are body-local Y).

Expected behavior: gear A turns at the commanded constant speed; gear B turns
in the opposite sense at a slower speed scaled by radA/radB, smoothly and
without divergence, for the whole run.
"""

import os
import math

import pychrono as chrono
import pychrono.irrlicht as chronoirr


# === Named constants === geometry, drive speed, and timing (all final values)
radA = 1.5                       # driving gear radius
radB = 3.5                       # driven gear radius
interaxis12 = radA + radB        # centre-to-centre distance (gears mesh)

truss_size = (15.0, 8.0, 2.0)    # fixed support box full extents
truss_density = 1000.0

gear_thickness = 0.5             # visual gear disc thickness
gear_density = 1000.0

motor_speed = 3.0                # commanded constant angular speed of gear A (rad/s)

shaft_radius = radA * 0.3        # visual shaft cylinder radius
shaft_length = 10.0              # visual shaft cylinder length

time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# Shafts run along world Y; the gear discs lie in the X-Z plane.
# A body-local frame whose +Z maps to world +Y aligns the gear/shaft spin axis.
q_axis_y = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)   # precomputed once

pos_gearA = chrono.ChVector3d(0, 0, -1)            # driving gear centre
pos_gearB = chrono.ChVector3d(interaxis12, 0, -2)  # driven gear centre

# === System & gravity === pure jointed MBS (no contact -> no collision system)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies === fixed truss + two gear discs + a visual shaft on gear A
mat = chrono.ChContactMaterialNSC()   # nominal material for the easy-body factories

truss = chrono.ChBodyEasyBox(truss_size[0], truss_size[1], truss_size[2],
                             truss_density, True, False, mat)
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetFixed(True)
sys.Add(truss)

# Driving gear A: a flat cylinder disc whose axis is world Y.
gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, gear_thickness,
                                  gear_density, True, False, mat)
gearA.SetPos(pos_gearA)
sys.Add(gearA)

# A slim visual shaft attached to gear A (a structural element, kept visible).
gearA.AddVisualShape(chrono.ChVisualShapeCylinder(shaft_radius, shaft_length),
                     chrono.ChFramed(chrono.VNULL,
                                     chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)))

# A contrasting radial spoke marks gear A so its rotation is plainly visible.
spokeA = chrono.ChVisualShapeBox(2.0 * radA * 0.9, gear_thickness * 1.2, radA * 0.18)
spokeA.SetColor(chrono.ChColor(0.9, 0.2, 0.2))
gearA.AddVisualShape(spokeA, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# Driven gear B: a larger flat cylinder disc on a parallel Y shaft.
gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, gear_thickness,
                                  gear_density, True, False, mat)
gearB.SetPos(pos_gearB)
sys.Add(gearB)

# A contrasting radial spoke marks gear B so its (slower) rotation is visible.
spokeB = chrono.ChVisualShapeBox(2.0 * radB * 0.9, gear_thickness * 1.2, radB * 0.14)
spokeB.SetColor(chrono.ChColor(0.2, 0.4, 0.9))
gearB.AddVisualShape(spokeB, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# === Joints / constraints === shaft bearings, drive motor, kinematic gear mesh
# Revolute bearing for gear A about its world-Y shaft (local +Z -> world +Y).
bearingA = chrono.ChLinkLockRevolute()
bearingA.Initialize(gearA, truss, chrono.ChFramed(pos_gearA, q_axis_y))
sys.AddLink(bearingA)

# Revolute bearing for gear B about its parallel world-Y shaft.
bearingB = chrono.ChLinkLockRevolute()
bearingB.Initialize(gearB, truss, chrono.ChFramed(pos_gearB, q_axis_y))
sys.AddLink(bearingB)

# Constant-speed motor driving gear A relative to the fixed truss.
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(gearA, truss, chrono.ChFramed(pos_gearA, q_axis_y))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(link_motor)

# Kinematic gear constraint A<->B. Shaft frames are BODY-LOCAL: each gear's
# spin axis is its local +Z mapped to world +Y, so use q_axis_y in body frames.
link_gear = chrono.ChLinkLockGear()
link_gear.Initialize(gearA, gearB, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), q_axis_y))
link_gear.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, q_axis_y))   # gear A local shaft frame
link_gear.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, q_axis_y))   # gear B local shaft frame
link_gear.SetTransmissionRatio(radA / radB)   # |omegaB/omegaA| = radA/radB
link_gear.SetEpicyclic(False)                 # ordinary (non-planetary) gear pair
link_gear.SetEnforcePhase(True)               # keep teeth phased over time
sys.AddLink(link_gear)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Spur gear pair driven by a rotational-speed motor")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(interaxis12 / 2, 5, -12), chrono.ChVector3d(interaxis12 / 2, 0, -1.5))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -5, 0),
                               chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render at a fixed cadence; advance physics between frames



frame = 0
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
    import traceback; traceback.print_exc()
    raise
finally:
    pass
