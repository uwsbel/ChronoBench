"""Slider-crank mechanism simulation (PyChrono 9.0.1, NSC, Irrlicht).

Models a classic planar crank-rod-slider linkage driven by a constant-speed
rotational motor:

  * a fixed truss/ground body that anchors the crank pivot and the slider guide,
  * a crank that rotates about a fixed ground pivot at the world origin,
  * a connecting rod pinned to the crank tip and to the piston,
  * a piston (slider) that translates along the world X guide.

Topology: crank-ground revolute + crank-ground rotation-speed motor, crank-rod
revolute, rod-piston revolute, and a piston-ground prismatic along X. The system
is a pure jointed multi-body assembly with no contact, so no collision system is
configured. The motor spins the crank at a constant angular speed; the piston
oscillates along X. Expected behavior: the piston position and speed trace out
the characteristic crank-slider curves as a function of the crank angle, sampled
over a 20-second run.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics of the crank-slider linkage
time_step = 1.0e-3          # integration step [s]
sim_end = 20.0             # stop the simulation after 20 seconds
render_fps = 50.0          # review render cadence [frames/s]

crank_radius = 1.0         # crank pin throw (pivot -> crank-rod pin) [m]
rod_length = 4.0           # connecting-rod length (crank pin -> piston pin) [m]
crank_speed = 2.0 * math.pi  # constant crank angular speed [rad/s] (1 rev/s)

crank_mass = 1.0           # crank mass [kg]
rod_mass = 1.0             # connecting-rod mass [kg]
piston_mass = 1.0          # piston mass [kg]

# Derived initial geometry (crank at angle 0 -> pin on +X axis).
crank_pin_x = crank_radius             # crank-rod pin world X at start [m]
piston_x0 = crank_radius + rod_length  # piston world X at start [m]

# === System & gravity === pure jointed MBS, gravity along -Y (planar XY motion)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed linkage with no contact/collision -> no SetCollisionSystemType.

# === Bodies === ground truss, crank, connecting rod, piston
# Ground / truss: fixed body anchoring the crank pivot and slider guide.
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
sys.AddBody(ground)
ground_pin = chrono.ChVisualShapeCylinder(0.1, 0.4)
ground.AddVisualShape(ground_pin, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# Crank: rotates about the ground pivot at the origin; origin at its own center,
# spanning from the pivot (local -crank_radius/2 on X) to the pin (local +).
crank = chrono.ChBody()
crank.SetMass(crank_mass)
crank.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
crank.SetPos(chrono.ChVector3d(crank_radius / 2.0, 0, 0))
crank.SetName("crank")
sys.AddBody(crank)
crank_vis = chrono.ChVisualShapeCylinder(0.08, crank_radius)
crank_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
crank.AddVisualShape(crank_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Connecting rod: pinned to the crank pin and to the piston; centered between them.
rod_center_x = (crank_pin_x + piston_x0) / 2.0
rod = chrono.ChBody()
rod.SetMass(rod_mass)
rod.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
rod.SetPos(chrono.ChVector3d(rod_center_x, 0, 0))
rod.SetName("rod")
sys.AddBody(rod)
rod_vis = chrono.ChVisualShapeCylinder(0.06, rod_length)
rod_vis.SetColor(chrono.ChColor(0.2, 0.5, 0.8))
rod.AddVisualShape(rod_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Piston (slider): translates along the world X guide.
piston = chrono.ChBody()
piston.SetMass(piston_mass)
piston.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
piston.SetPos(chrono.ChVector3d(piston_x0, 0, 0))
piston.SetName("piston")
sys.AddBody(piston)
piston_vis = chrono.ChVisualShapeBox(0.5, 0.5, 0.5)
piston_vis.SetColor(chrono.ChColor(0.3, 0.7, 0.3))
piston.AddVisualShape(piston_vis)

# === Joints / constraints === revolutes + prismatic + speed motor
# Crank-ground revolute hinge: planar XY motion -> hinge axis is world +Z (QUNIT).
crank_pivot = chrono.ChLinkLockRevolute()
crank_pivot.Initialize(crank, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(crank_pivot)

# Crank-ground rotation-speed motor: spins the crank at a constant angular speed.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(crank_speed))
sys.AddLink(motor)

# Crank-rod revolute at the crank pin (crank far end / rod near end), hinge +Z.
crank_rod = chrono.ChLinkLockRevolute()
crank_rod.Initialize(crank, rod, True,
                     chrono.ChFramed(chrono.ChVector3d(crank_radius / 2.0, 0, 0), chrono.QUNIT),   # crank far end (local +X)
                     chrono.ChFramed(chrono.ChVector3d(-rod_length / 2.0, 0, 0), chrono.QUNIT))    # rod near end (local -X)
sys.AddLink(crank_rod)

# Rod-piston revolute at the piston pin (rod far end / piston center), hinge +Z.
rod_piston = chrono.ChLinkLockRevolute()
rod_piston.Initialize(rod, piston, True,
                      chrono.ChFramed(chrono.ChVector3d(rod_length / 2.0, 0, 0), chrono.QUNIT),    # rod far end (local +X)
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))                   # piston center
sys.AddLink(rod_piston)

# Piston-ground prismatic guide along world X: map frame local +Z onto the X axis.
piston_guide = chrono.ChLinkLockPrismatic()
piston_guide.Initialize(piston, ground,
                        chrono.ChFramed(chrono.ChVector3d(piston_x0, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(piston_guide)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y -> Y is up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, 4.0, 8.0), chrono.ChVector3d(2.5, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -1.0, 0),
                               chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid in the XZ plane below

# === Main loop === drive the crank, advance physics, sample piston motion
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# Cache joint/body handles fetched once and reused every step.
piston_ref = piston            # cache: piston handle, reused for pose each step
motor_ref = motor              # cache: motor handle, reused for crank angle each step



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
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state mid-run
    import traceback
    traceback.print_exc()
    raise
