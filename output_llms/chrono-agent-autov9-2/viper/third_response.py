"""Viper rover driving straight on SCM deformable (Bekker-Wong) soft-soil terrain.

Model:
  - System type: NSC (ChSystemNSC) with Bullet collision, Z-up world, gravity -9.81 m/s^2.
  - Protagonist: the four-wheeled Viper rover (pychrono.robot) driven by a
    ViperDCMotorControl driver. The DC-motor driver spins all four wheels; the
    steering angle is held constant at 0.0 rad so the rover travels in a straight line.
  - Terrain: a deformable SCM (Soil Contact Model) patch from pychrono.vehicle,
    replacing any rigid ground. The wheels sink into and leave ruts in the soil.

Expected behavior:
  The rover's wheels grip the soft soil and the chassis translates forward in a
  straight line (no turning), sinking slightly into the deformable terrain over the
  simulation. The chassis X-position should grow monotonically while Y stays ~0.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / timing) ===
time_step = 1e-3                       # solver step (s)
sim_end = 12.0                         # simulation duration (s)
render_fps = 50.0                      # review render cadence (frames/s)

# SCM terrain extents and soil grid
terrain_length = 14.0                  # X extent of the soft-soil patch (m)
terrain_width = 8.0                    # Y extent of the soft-soil patch (m)
terrain_resolution = 0.04              # SCM grid spacing (m) — fine enough for visible ruts

# Rover spawn (front-of-patch so it has room to drive forward in +X)
rover_init_x = -terrain_length / 2.0 + 2.0      # start near the -X edge
rover_init_y = 0.0
rover_init_z = 0.30                    # chassis reference height above the soil rest plane
steering_angle = 0.0                   # constant steering -> straight-line driving (rad)

# Derived render cadence (precomputed once; never recomputed in the loop)
render_every = max(1, round(1.0 / (render_fps * time_step)))           # precomputed once


# === System & gravity === NSC system with Bullet collision (rover wheels contact soil)
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)       # contact: wheels <-> SCM soil
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Tight collision envelope/margin keeps the small rover-wheel contacts accurate.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Terrain === SCM deformable soft soil (Bekker-Wong); replaces any rigid ground
# SCMTerrain requires the collision system to exist first (set above).
terrain = veh.SCMTerrain(sys)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi   — frictional modulus (Pa)
    0,      # Bekker_Kc     — cohesive modulus (Pa)
    1.1,    # Bekker_n      — sinkage exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear deformation modulus (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R     — vertical damping (Pa*s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)   # color the ruts by sinkage
terrain.SetMeshWireframe(False)
# NOTE: no AddActiveDomain — the full patch stays active so every wheel always casts
# soil rays and the rover keeps traction (a small moving patch can starve the wheels).
terrain.Initialize(terrain_length, terrain_width, terrain_resolution)

# === Rover === Viper four-wheeled rover with DC-motor speed/steering driver
driver = robot.ViperDCMotorControl()                 # spins all wheels; holds a steering angle
rover = robot.Viper(sys)
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(
    chrono.ChVector3d(rover_init_x, rover_init_y, rover_init_z),
    chrono.ChQuaterniond(1, 0, 0, 0)))
driver.SetSteering(steering_angle)                   # constant 0.0 rad -> drive straight

chassis = rover.GetChassis()                         # cache: fetched once, reused every step
assert rover_init_z > 0.0, "rover must spawn above the SCM rest plane (z=0)"

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - SCM deformable terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(rover_init_x - 2.0, -3.0, 2.0),
              chrono.ChVector3d(rover_init_x, 0, 0.3))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics advanced in inner batches

try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            driver.SetSteering(steering_angle)   # keep steering straight every step
            rover.Update()                       # advance the DC-motor driver / wheel motors
            terrain.Synchronize(t)
            sys.DoStepDynamics(time_step)
            terrain.Advance(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state during stepping
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
