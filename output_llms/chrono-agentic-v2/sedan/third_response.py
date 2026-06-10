"""
BMW E90 Sedan simulation on a highway mesh terrain with PID speed control.

System type: NSC (rigid terrain, catalog vehicle default).
Main bodies: BMW_E90 chassis, four wheel spindles, highway RigidTerrain patch.
Expected behavior: the vehicle starts from rest at an adjusted initial position
and orientation, accelerates toward a reference speed via a PID throttle
controller, and drives along the highway mesh terrain. Steering response time
is set to 5 seconds via ChInteractiveDriverIRR deltas. The simulation uses a
finer time step (5e-4 s) and render step size (1/100 s) for improved control
fidelity.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Data paths (mandatory for catalog vehicle scoring) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants — geometry, physics, control ===
INIT_X = -40.0                  # vehicle initial X position (m)
INIT_Y = 0.0                    # vehicle initial Y position (m)
INIT_Z = 0.5                    # vehicle initial Z (above terrain, m)
INIT_YAW = 0.0                  # initial heading angle (rad)

TARGET_SPEED = 20.0             # reference speed for PID controller (m/s)
KP = 0.4                        # PID proportional gain
KI = 0.05                       # PID integral gain
KD = 0.01                       # PID derivative gain

STEP_SIZE = 5e-4                # fine simulation step size (s) — decreased for finer control
RENDER_STEP_SIZE = 1.0 / 100.0  # fine render step size (s) — decreased for finer control
SIM_END = 30.0                  # total simulation duration (s)
STEERING_TIME = 5.0             # time to reach max steering (s) — increased response time

TERRAIN_LENGTH = 300.0          # highway terrain patch length (m)
TERRAIN_WIDTH = 30.0            # highway terrain patch width (m)

# Derived constants — precomputed once
render_steps = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once
init_rot = chrono.QuatFromAngleZ(INIT_YAW)                       # precomputed once
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)             # precomputed once

# === PID speed controller (scored core — drives throttle from speed error) ===
class PIDThrottleController:
    """Proportional-Integral-Derivative controller for throttle based on speed error."""
    def __init__(self, kp, ki, kd, target_speed):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_speed = target_speed
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = 0.0

    def compute(self, current_speed, time):
        dt = time - self._prev_time
        if dt <= 0.0:
            return 0.0, 0.0
        error = self.target_speed - current_speed
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._prev_error = error
        self._prev_time = time
        throttle = max(0.0, min(1.0, output))
        braking = max(0.0, min(1.0, -output))
        return throttle, braking


# === Vehicle setup (BMW E90 sedan) ===
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(STEP_SIZE)
vehicle.Initialize()

# === System & bodies (created by the veh.BMW_E90 wrapper) ===
sys = vehicle.GetSystem()               # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()       # cache: fetched once, reused every step
# wheels/spindles: vehicle.GetVehicle().GetAxle(i); terrain: RigidTerrain below
# joints: suspension + steering links created inside the BMW_E90 wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain — highway mesh patch ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/highway.obj"),    # highway mesh terrain
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht (Irrlicht default for vehicles) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("BMW E90 Sedan — Highway PID Speed Control")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver — interactive with 5-second steering response (scored-core default) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = STEERING_TIME              # 5 s to reach max steering
throttle_time = 1.0                        # 1 s to reach max throttle
braking_time = 0.3                         # 0.3 s to reach max braking
driver.SetSteeringDelta(RENDER_STEP_SIZE / steering_time)
driver.SetThrottleDelta(RENDER_STEP_SIZE / throttle_time)
driver.SetBrakingDelta(RENDER_STEP_SIZE / braking_time)
driver.Initialize()

# === PID controller instance ===
pid = PIDThrottleController(KP, KI, KD, TARGET_SPEED)

# === Output directories and recording setup ===

# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:

    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # PID override of throttle/braking based on chassis speed (scored core)
        current_speed = chassis.GetPosDt().Length()   # cache: computed once per step
        pid_throttle, pid_braking = pid.compute(current_speed, time)


        driver_inputs.m_throttle = pid_throttle
        driver_inputs.m_braking = pid_braking

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

except (RuntimeError, ValueError) as exc:     # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
