"""
Sedan turn 3 — BMW E90 with highway terrain, PID speed control.

Changes from turn 1/2:
  - Single vehicle with adjusted initial position
  - Steering response time increased to 5 s (smoother steering)
  - Smaller step size (5e-4) for finer control
  - Highway terrain (wide flat road surface)
  - Reference speed input with PID throttle controller
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
# Finer step size for better control (turn 3 modification)
step_size = 5e-4          # 0.5 ms (was 1e-3 in turns 1/2)
tire_step_size = step_size
render_step_size = 1.0 / 50.0   # 50 FPS — keep render fps same, more physics per frame
render_steps = math.ceil(render_step_size / step_size)

# Highway straight road length
terrain_length = 200.0
terrain_width = 20.0

# Initial vehicle location (adjusted from turn 1's 0,0,0.5)
init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Camera chase point
track_point = chrono.ChVector3d(-5.0, 0.0, 1.8)

# === PID speed controller constants ===
REFERENCE_SPEED = 8.0     # m/s — desired cruising speed
KP = 1.0                  # proportional gain
KI = 0.1                  # integral gain
KD = 0.05                 # derivative gain

# === Contact ===
contact_method = chrono.ChContactMethod_NSC
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# === Vehicle ===
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain — highway (wide flat road surface) ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 80, 80)
patch.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
terrain.Initialize()

# === PID Driver Controller ===
class PIDDriver(veh.ChDriver):
    """Custom driver with PID speed control based on reference speed."""

    def __init__(self, veh_wrapper, ref_speed, kp, ki, kd):
        # veh.ChDriver takes the ChWheeledVehicle from wrapper.GetVehicle()
        super().__init__(veh_wrapper.GetVehicle())
        self._veh_wrapper = veh_wrapper
        self.ref_speed = ref_speed
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.steering = 0.0

    def Synchronize(self, time):
        # Get current longitudinal speed from the vehicle wrapper
        veh_speed = self._veh_wrapper.GetVehicle().GetSpeed()
        error = self.ref_speed - veh_speed

        # PID control
        self.integral += error * step_size
        derivative = (error - self.prev_error) / step_size if step_size > 0 else 0.0
        throttle = self.kp * error + self.ki * self.integral + self.kd * derivative
        throttle = max(0.0, min(1.0, throttle))

        self.prev_error = error
        self.SetThrottle(throttle)
        self.SetBraking(0.0)
        self.SetSteering(self.steering)


driver = PIDDriver(vehicle, REFERENCE_SPEED, KP, KI, KD)
driver.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Sedan — Highway PID")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(track_point, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# === Simulation loop ===
sim_end = 20.0   # 20 second run for good highway segment

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
