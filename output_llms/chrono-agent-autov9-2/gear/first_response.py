"""Epicyclic gear train driven by a constant-speed motor (PyChrono, NSC).

This script models a planetary (epicyclic) gear system:
  * a fixed TRUSS (ground) that anchors the mechanism,
  * a rotating BAR (the planet carrier) that pivots about the central axis,
  * a central SUN gear and an orbiting PLANET gear,
  * a rotational-speed MOTOR enforcing a constant spin on the sun gear.

The two gears are coupled by a kinematic ChLinkLockGear constraint with the
epicyclic flag enabled, so the planet rolls around the sun while the carrier bar
sweeps. There is NO collision/contact in this mechanism — every interaction is a
joint or a kinematic gear constraint — so no collision system is configured
(pure jointed multi-body system).

System type: ChSystemNSC. Gravity acts along world -Z (Z-up world). The expected
behavior is steady, constant-rate rotation of the sun gear with the planet gear
and carrier bar in continuous coupled motion (no transients, no divergence).
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics parameters (no bare literals downstream)
time_step = 1e-3            # integration step [s]
sim_end = 8.0               # simulation duration [s]
render_fps = 50.0           # review render cadence [frames/s]

motor_speed = math.pi       # constant sun-gear angular speed [rad/s] (~0.5 rev/s)

sun_radius = 1.0            # sun (central) gear pitch radius [m]
planet_radius = 0.5         # planet (orbiting) gear pitch radius [m]
gear_thickness = 0.3        # axial thickness of each gear wheel [m]
gear_density = 1000.0       # gear material density [kg/m^3]

truss_size = 0.4            # truss/support cube edge [m]
bar_length = sun_radius + planet_radius   # carrier-bar span from sun axis to planet axis [m]
bar_section = 0.18          # carrier-bar cross-section [m]

# Spin axis of every wheel is world +Y (gears stand upright in the Z-up world).
spin_axis = chrono.ChVector3d(0, 1, 0)
# Map a joint/shaft frame local +Z onto world +Y so the constraint axis is the spin axis.
# precomputed once: reused by both revolutes and both gear shaft frames.
q_z_to_y = chrono.Q_ROTATE_Z_TO_Y

# Derived world positions (computed once from the constants above).
sun_center = chrono.ChVector3d(0, 0, 0)
planet_center = chrono.ChVector3d(bar_length, 0, 0)
bar_center = chrono.ChVector3d(bar_length / 2.0, -gear_thickness, 0)  # behind the gears in Y

# === System & gravity === single NSC system; gravity along -Z (Z-up)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# NOTE: pure jointed/kinematic mechanism (motor + revolutes + gear link), no contact,
# so SetCollisionSystemType is intentionally omitted (truth omits it for gear trains).

# === Bodies === fixed truss, carrier bar, sun gear, planet gear (all visualized)
# Fixed truss / support — anchors the central axis and carries the motor reaction.
truss = chrono.ChBodyEasyBox(truss_size, truss_size, truss_size, gear_density, True, False)
truss.SetPos(chrono.ChVector3d(0, -2.0 * gear_thickness, 0))
truss.SetFixed(True)
truss.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.25))
sys.Add(truss)

# Rotating bar = planet carrier, spanning from the sun axis out to the planet axis.
bar = chrono.ChBodyEasyBox(bar_length, bar_section, bar_section, gear_density, True, False)
bar.SetPos(bar_center)
bar.GetVisualShape(0).SetColor(chrono.ChColor(0.25, 0.25, 0.3))
sys.Add(bar)

# Sun gear (central, motor-driven). Cylinder axis along Y = the spin axis.
sun_gear = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, sun_radius, gear_thickness, gear_density, True, False)
sun_gear.SetPos(sun_center)
sun_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.3, 0.2))
sys.Add(sun_gear)

# Planet gear (orbits the sun, mounted at the far end of the carrier bar).
planet_gear = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, planet_radius, gear_thickness, gear_density, True, False)
planet_gear.SetPos(planet_center)
planet_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.5, 0.8))
sys.Add(planet_gear)

# === Joints / constraints === revolutes to ground/bar, motor, and the gear couple
# Carrier bar pivots about the central (sun) axis relative to the fixed truss.
rev_bar = chrono.ChLinkLockRevolute()
rev_bar.Initialize(bar, truss, chrono.ChFramed(sun_center, q_z_to_y))
sys.AddLink(rev_bar)

# Sun gear pivots about the same central axis relative to the truss.
rev_sun = chrono.ChLinkLockRevolute()
rev_sun.Initialize(sun_gear, truss, chrono.ChFramed(sun_center, q_z_to_y))
sys.AddLink(rev_sun)

# Planet gear pivots about its own axis relative to the rotating carrier bar.
rev_planet = chrono.ChLinkLockRevolute()
rev_planet.Initialize(planet_gear, bar, chrono.ChFramed(planet_center, q_z_to_y))
sys.AddLink(rev_planet)

# Motor enforces a constant rotation speed of the sun gear relative to the truss.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sun_gear, truss, chrono.ChFramed(sun_center, q_z_to_y))
motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(motor)

# Kinematic gear coupling between sun and planet. Shaft frames are BODY-LOCAL,
# mapping each body's local +Z onto its spin axis (world +Y). The epicyclic flag
# tells the constraint the planet axis travels with the moving carrier bar.
gear_link = chrono.ChLinkLockGear()
gear_link.Initialize(sun_gear, planet_gear, chrono.ChFramed(sun_center, chrono.QUNIT))
gear_link.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, q_z_to_y))   # sun spin axis (local +Z -> +Y)
gear_link.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, q_z_to_y))   # planet spin axis (local +Z -> +Y)
gear_link.SetTransmissionRatio(planet_radius / sun_radius)          # |w_planet| / |w_sun|
gear_link.SetEpicyclic(True)                                        # planet orbits on the carrier
gear_link.SetEnforcePhase(True)                                     # keep meshed teeth in phase
sys.AddLink(gear_link)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Epicyclic Gear Train")
vis.Initialize()                                    # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, -4.0, 2.5), chrono.ChVector3d(bar_length / 2.0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -sun_radius - 0.5), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop, physics in inner batch
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once
get_time = sys.GetChTime                                       # cache: getter fetched once, reused per step


try:

    frame = 0
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
finally:
    pass

# === Post-processing === assemble review video + plot, then clean frames
