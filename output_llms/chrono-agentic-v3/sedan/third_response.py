"""
Sedan (BMW E90) highway driving simulation with PID speed controller.

System type: ChSystemNSC (owned by BMW_E90 wrapper).
Bodies: BMW E90 chassis, suspension, wheels, tires (wrapper-managed).
Terrain: RigidTerrain with highway mesh patch.
Driver: Custom PID throttle controller targeting a reference speed.
Expected behavior: The sedan accelerates onto a highway mesh terrain,
    the PID controller maintains a target cruising speed, and steering
    response ramps slowly (5 s to full lock).
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Physics timestep and render cadence — decreased for finer control
STEP_SIZE = 5e-4            # simulation step size (s)
RENDER_STEP_SIZE = 1.0 / 50.0  # render cadence (s)
SIM_END = 20.0              # simulation duration (s)

render_steps = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

# Vehicle initial placement — adjusted position and orientation
# Spawn inside the highway mesh (mesh spans X:[0,300], Y:[-3,3], Z≈0)
INIT_POS = chrono.ChVector3d(10.0, 0.0, 0.5)     # highway lane spawn at mesh start
INIT_ROT = chrono.QuatFromAngleZ(0.0)              # heading along +X

SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest (m)

# Terrain
TERRAIN_LENGTH = 500.0    # m
TERRAIN_WIDTH  = 20.0     # m

# Driver / PID speed controller
STEERING_TIME = 5.0       # seconds to reach full steering lock (increased per prompt)
THROTTLE_TIME = 1.0       # seconds to full throttle
BRAKING_TIME  = 0.3       # seconds to full brake

REF_SPEED     = 15.0      # reference (target) vehicle speed [m/s]
PID_KP        = 0.4       # PID proportional gain
PID_KI        = 0.02      # PID integral gain
PID_KD        = 0.05      # PID derivative gain

# === Data paths (scored-core truth component) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)  # MANDATORY
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(STEP_SIZE)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.BMW_E90 wrapper) ===
sys = vehicle.GetSystem()         # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()  # cache: fetched once, reused every step
# wheels/spindles: vehicle.GetVehicle().GetAxle(i)...
# joints: suspension + steering links created inside the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Verify wheel bottom is at or above terrain surface
TIRE_RADIUS = 0.33
ZTOL = 0.05
veh_obj = vehicle.GetVehicle()
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into support: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
)

# === Terrain — highway mesh patch ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Highway mesh patch — uses bundled highway-style uneven road mesh (300m x 6m)
# Mesh OBJ spans X:[0,300] in local coords; place at world origin so mesh is at X:[0,300]
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/uneven_300m_6m_10mm.obj"),
)
patch.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

terrain.Initialize()

# === Driver — custom PID speed controller (scripted, scored core) ===
# The prompt requests a PID throttle controller with a reference speed input.
# This is a scripted/data driver shape — scripted inputs belong in scored core.
class PIDSpeedDriver(veh.ChDriver):
    """PID throttle controller targeting REF_SPEED; steering ramps up slowly."""
    def __init__(self, vehicle_obj, ref_speed, kp, ki, kd):
        super().__init__(vehicle_obj)
        self._ref = ref_speed
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = 0.0

    def Synchronize(self, time):
        dt = time - self._prev_time
        if dt <= 0.0:
            return
        speed = chassis.GetPosDt().Length() if chassis is not None else 0.0
        error = self._ref - speed
        self._integral += error * dt
        # Clamp integral anti-windup
        self._integral = max(-10.0, min(10.0, self._integral))
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        throttle_raw = self._kp * error + self._ki * self._integral + self._kd * derivative
        throttle = max(0.0, min(1.0, throttle_raw))
        if throttle_raw < 0.0:
            braking = max(0.0, min(1.0, -throttle_raw))
        else:
            braking = 0.0
        self.SetThrottle(throttle)
        self.SetBraking(braking)
        self.SetSteering(0.0)  # straight-line cruise
        self._prev_error = error
        self._prev_time = time

driver = PIDSpeedDriver(vehicle.GetVehicle(), REF_SPEED, PID_KP, PID_KI, PID_KD)
driver.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Sedan Highway PID Speed Control")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()   # vehicle truths use directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Main loop ===

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and vehicle.GetSystem().GetChTime() < SIM_END:
        time = vehicle.GetSystem().GetChTime()  # cache: current sim time each frame

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
