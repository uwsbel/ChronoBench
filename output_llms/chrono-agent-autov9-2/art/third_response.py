"""ARTcar straight-line acceleration on flat rigid terrain (PyChrono 9.0.x, Irrlicht).

Models the small-scale ART (Autonomous Racing Tatra-like) RC car catalog vehicle
(`veh.ARTcar`, an NSC wrapper-managed wheeled vehicle) accelerating from rest
along +X on a wide flat RigidTerrain patch. The car is tuned for higher top speed
via its electric-drive setters: max motor voltage ratio 0.26, stall torque 0.4 N*m,
and a reduced tire rolling resistance of 0.03. A scripted ChDriver applies a short
settle phase followed by full throttle and zero steering, so the expected behavior
is a monotonically increasing forward speed and a straight trajectory down +X.

System: NSC (the ARTcar wrapper owns its ChSystemNSC). Main bodies: chassis +
four wheels/spindles created by the wrapper, plus a rigid terrain patch body.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 1.0e-3                  # integration step (s)
TIRE_STEP = 1.0e-3                  # tire force-model step (s)
SIM_END = 12.0                      # simulated duration (s)
RENDER_FPS = 50.0                   # review-video frame rate

TERRAIN_LENGTH = 200.0             # X extent of the rigid patch (m) — wide (>=160 m)
TERRAIN_WIDTH = 40.0               # Y extent of the rigid patch (m)
TERRAIN_FRICTION = 0.9             # patch friction coefficient
TERRAIN_RESTITUTION = 0.01         # patch restitution

# Tuned ARTcar drive parameters (final desired values for a faster vehicle).
MAX_MOTOR_VOLTAGE_RATIO = 0.26     # higher voltage ratio -> more drive power
STALL_TORQUE = 0.4                 # motor stall torque (N*m)
TIRE_ROLLING_RESISTANCE = 0.03     # lower rolling resistance -> less drag

INIT_X = -TERRAIN_LENGTH / 2.0 + 10.0   # spawn 10 m in from the patch's -X edge
INIT_Y = 0.0
INIT_Z = 0.2                       # chassis-origin height so wheels rest on z=0
TIRE_RADIUS = 0.103                # ARTcar tire radius (m), for the footprint assert
ZTOL = 0.05                        # allowed wheel-bottom clearance vs terrain top

SETTLE_TIME = 0.5                  # brake-hold settle before launch (s)
DRIVE_THROTTLE = 1.0               # full throttle after settle

# === Driver === scripted, time-based open-loop control (no human-in-the-loop)
class StraightLineDriver(veh.ChDriver):
    """Hold brakes briefly to settle on the terrain, then full throttle, no steering."""

    def __init__(self, vehicle, settle_time, throttle):
        super().__init__(vehicle)
        self._settle_time = settle_time     # cache: launch threshold reused every step
        self._throttle = throttle           # cache: target throttle reused every step

    def Synchronize(self, time):
        if time < self._settle_time:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(self._throttle)
            self.SetBraking(0.0)
        self.SetSteering(0.0)


# === Vehicle === ARTcar wrapper owns the ChSystemNSC; set collision before Initialize
vehicle = veh.ARTcar()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetTireType(veh.TireModelType_TMEASY)        # force-element tire on rigid road
vehicle.SetTireStepSize(TIRE_STEP)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT))

# Tuning setters that make the car faster (final requested values).
vehicle.SetMaxMotorVoltageRatio(MAX_MOTOR_VOLTAGE_RATIO)
vehicle.SetStallTorque(STALL_TORQUE)
vehicle.SetTireRollingResistance(TIRE_ROLLING_RESISTANCE)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.ARTcar wrapper) ===
sys = vehicle.GetSystem()                      # cache: ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()             # cache: main chassis rigid body, reused every step
veh_obj = vehicle.GetVehicle()                 # cache: ChWheeledVehicle handle, reused below
# wheels/spindles: veh_obj.GetAxles()[i] (created inside the wrapper)
# joints: suspension + steering links created inside the wrapper

# Framework requirement: explicitly select the Bullet collision system AFTER Initialize.
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === wide flat rigid patch the car drives along
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Verify the wheels rest on (not through) the terrain top after Initialize.
spindle_world = [veh_obj.GetSpindlePos(axle, side)
                 for axle in range(veh_obj.GetNumberAxles())
                 for side in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} vs top z=0.0; "
    f"raise INIT_Z by {-wheel_bottom_z:.3f} m"
)

# === Driver === scripted launch controller
driver = StraightLineDriver(veh_obj, SETTLE_TIME, DRIVE_THROTTLE)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht window with chase camera + lights + sky
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar straight-line acceleration")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.4), 2.5, 0.4)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Precomputed loop constants === computed once before the loop
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# === Main loop === render-cadence outer loop; Synchronize/Advance the subsystem stack

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)        # advances the wrapper-owned ChSystem
            vis.Advance(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + plot, then clean up frames
