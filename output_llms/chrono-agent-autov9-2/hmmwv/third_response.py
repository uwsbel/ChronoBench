"""HMMWV full-model wheeled vehicle driven by a scripted custom driver on flat rigid terrain.

Model
-----
- System: a single ChSystemSMC owned by the veh.HMMWV_Full wrapper (SMC contact,
  consistent with the SMC contact materials used for the terrain patch and tires).
- Bodies: the HMMWV chassis, four spindles/wheels with TMEASY tires (created inside
  the wrapper), and one flat RigidTerrain patch acting as the ground.
- Control: a custom veh.ChDriver subclass (MyDriver) computes throttle, steering and
  braking as explicit functions of simulation time. It holds all inputs at zero for an
  initial delay, then ramps the throttle up to its hold value, and applies a sinusoidal
  steering oscillation once the steering phase begins.

Expected behavior
-----------------
The vehicle stays put during the input delay, then accelerates forward as the throttle
ramps in, and begins weaving left/right once the sinusoidal steering activates. The run
stops when the simulation clock reaches the end time.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Named constants === geometry / physics / control schedule (no bare literals downstream)
time_step = 1e-3                       # integration step (s)
sim_end = 4.0                          # end the simulation at 4 s
render_fps = 50.0                      # review-video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once: physics steps per frame

DRIVER_DELAY = 0.5                     # s: hold all inputs at zero until this time
THROTTLE_HOLD = 0.7                    # target throttle after the ramp
THROTTLE_RAMP = 0.2                    # s: reach THROTTLE_HOLD this long after the delay
STEER_START = 2.0                      # s: sinusoidal steering begins at this time
STEER_AMP = 0.5                        # steering amplitude (-1..+1)
STEER_FREQ = 0.5                       # steering oscillation frequency (Hz-like rate)

TERRAIN_LENGTH = 100.0                 # m: rigid terrain patch X size
TERRAIN_WIDTH = 100.0                  # m: rigid terrain patch Y size
TERRAIN_FRICTION = 0.9                 # tire-ground friction coefficient
TERRAIN_YOUNG = 2e7                    # Pa: SMC contact stiffness

INIT_X, INIT_Y = 0.0, 0.0              # spawn XY on the (centered) terrain patch
SUSPENSION_REF_HEIGHT = 0.5            # chassis-origin height above wheel-bottom at rest
TERRAIN_TOP_Z = 0.0                    # flat terrain top is at z = 0
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT     # derived chassis spawn height
TIRE_RADIUS = 0.464                    # HMMWV tire radius (m), for the footprint assert
ZTOL = 0.1                             # allowed wheel-bottom clearance/overlap vs terrain


# === Custom driver === scripted, time-based control law (throttle ramp + sinusoidal steer)
class MyDriver(veh.ChDriver):
    """Open-loop driver: zero inputs during a delay, a throttle ramp, then sinusoidal steering."""

    def __init__(self, vehicle, delay):
        super().__init__(vehicle)
        self.delay = delay                                 # input delay (s)

    def Synchronize(self, time):
        # Hold everything at zero until the delay elapses (a delay in driver inputs).
        if time < self.delay:
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.0)
            return

        # Throttle ramps linearly from 0 to THROTTLE_HOLD over THROTTLE_RAMP after the delay.
        ramp_t = time - self.delay
        if ramp_t < THROTTLE_RAMP:
            throttle = THROTTLE_HOLD * (ramp_t / THROTTLE_RAMP)
        else:
            throttle = THROTTLE_HOLD
        self.SetThrottle(throttle)
        self.SetBraking(0.0)

        # Sinusoidal steering pattern, active only from STEER_START onward.
        if time >= STEER_START:
            self.SetSteering(STEER_AMP * math.sin(2.0 * math.pi * STEER_FREQ * (time - STEER_START)))
        else:
            self.SetSteering(0.0)


# === Vehicle === HMMWV full model; the wrapper creates and owns the ChSystem + bodies
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)            # grip-curve tire so the vehicle drives
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                  # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()            # cache: main chassis rigid body, reused every step
vehicle = hmmwv.GetVehicle()                # cache: ChWheeledVehicle, reused for spindle queries
# wheels/spindles: vehicle.GetSpindlePos(axle, side); joints: suspension + steering links inside wrapper

# The scene has tire/terrain contact -> Bullet collision is required.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === flat rigid ground patch the wheels roll on
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(TERRAIN_YOUNG)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Footprint check === assert the wheels rest on (not through) the terrain after Initialize
spindle_world = []
for axle in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(vehicle.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === custom scripted driver, initialized with the requested delay parameter
driver = MyDriver(vehicle, DRIVER_DELAY)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase camera + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV - scripted custom driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))     # ground reference grid
vis.AttachVehicle(vehicle)
vis.AttachDriver(driver)                       # steering/throttle/brake HUD bars

# === Main loop === render-cadence outer loop; full vehicle subsystem stack advanced inline
try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(time_step)
            terrain.Advance(time_step)
            hmmwv.Advance(time_step)          # advances the wrapper-owned system
            vis.Advance(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / invalid state mid-run
    import traceback
    traceback.print_exc()
    raise
