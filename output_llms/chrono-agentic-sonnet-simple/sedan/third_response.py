import os
import math
import pychrono as chrono
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

# Vehicle initial position and orientation (adjusted per prompt)
# Highway mesh runs Y: -75 to +75 (length), X: -11 to +11 (width) — vehicle drives along +Y
init_loc = chrono.ChVector3d(0.0, -60.0, 0.5)                       # near start of highway, centered in lane
init_rot = chrono.QuatFromAngleZ(math.pi / 2)                        # facing +Y direction (along highway)

step_size = 5e-4                                                     # decreased step size for finer control
tire_step_size = 5e-4                                                # decreased tire step size
render_step_size = 1.0 / 50.0                                        # 50 fps render
sim_end = 20.0                                                       # simulation end time (s)

target_speed = 20.0                                                  # reference speed (m/s, ~72 km/h)

# BMW E90 sedan wrapper
sedan = veh.BMW_E90()
sedan.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
sedan.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
sedan.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tires for highway
sedan.SetTireStepSize(tire_step_size)
sedan.Initialize()

system = sedan.GetSystem()                                           # wrapper owns the system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", sedan.GetVehicle().GetMass())                # report total vehicle mass

sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan.SetTireVisualizationType(veh.VisualizationType_MESH)

# Rigid terrain with highway mesh
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()                            # NSC material matches contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Highway mesh patch — collision mesh with road texture
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    veh.GetDataFile('terrain/meshes/Highway_col.obj'),               # collision mesh
)
patch.SetTexture(veh.GetDataFile('terrain/textures/concrete.jpg'), 60, 20)  # road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# Irrlicht visualization (vehicle-specific)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan Highway PID Speed Control')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)    # chase cam above/behind chassis
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(sedan.GetVehicle())

# PID controller driver for throttle based on speed error
class PIDSpeedDriver(veh.ChDriver):
    def __init__(self, vehicle, target_speed, Kp=0.4, Ki=0.0, Kd=0.0):
        super().__init__(vehicle)
        self.target_speed = target_speed                             # reference speed
        self.Kp = Kp                                                 # proportional gain
        self.Ki = Ki                                                 # integral gain
        self.Kd = Kd                                                 # derivative gain
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = 0.0

    def Synchronize(self, time):
        speed = sedan.GetVehicle().GetSpeed()                        # current vehicle speed
        error = self.target_speed - speed                            # speed error
        dt = time - self._prev_time if time > self._prev_time else 1e-4
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        throttle = self.Kp * error + self.Ki * self._integral + self.Kd * derivative
        throttle = max(0.0, min(1.0, throttle))                      # clamp 0..1
        braking = max(0.0, min(1.0, -self.Kp * error))              # brake when speed exceeds target
        self.SetThrottle(throttle)
        self.SetBraking(braking)
        self.SetSteering(0.0)                                        # straight ahead
        self._prev_error = error
        self._prev_time = time

driver = PIDSpeedDriver(sedan.GetVehicle(), target_speed)

# Steering response time 5s (as per prompt, increased)
steering_time = 5.0                                                  # seconds to reach full steering
throttle_time = 1.0                                                  # seconds to reach full throttle
braking_time = 0.3                                                   # seconds to reach full braking
driver.Initialize()

render_steps = math.ceil(render_step_size / step_size)               # steps per rendered frame
render_every = max(1, round(1.0 / (50.0 * step_size)))               # cadence constant (untagged)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    sedan.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)


    driver.Advance(step_size)
    terrain.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)
