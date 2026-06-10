"""HMMWV wheeled vehicle on flat rigid terrain with a ROS-shaped publishing layer.

Model
-----
- System: SMC (the HMMWV_Full wrapper owns a ChSystemSMC) with Bullet collision.
- Main bodies: HMMWV chassis + four wheels/spindles (created by the wrapper),
  a flat RigidTerrain patch, and a fixed visualization box used as a lidar target.
- Actuation: an open-loop ChDataDriver applies throttle/steering so the vehicle
  accelerates forward and drives across the terrain.
- Sensing: a ChLidarSensor mounted on the chassis scans the scene; its range data
  is consumed by a ROS-shaped lidar handler.

ROS layer (reconstructed in plain Python — there is NO pychrono.ros module here)
-------------------------------------------------------------------------------
The ROS interface is modeled after Chrono's ChROSHandler / ChROSManager design
without importing any ROS package:
- ChROSHandler is a rate-gated base: Update(time) fires Tick(time) only when the
  configured publish rate has elapsed.
- VehicleStatePublisher (a handler) "publishes" chassis pose + speed each tick.
- DriverInputsHandler (a handler) "publishes" the current steering/throttle/braking.
- LidarHandler (a handler) "publishes" the most-recent lidar range buffer.
- ChROSPythonManager owns the handlers and is ticked once per physics step.

Expected behavior
------------------
The vehicle starts on the terrain (asserted via the spindle world positions) and
drives forward under throttle; the chassis X position increases monotonically and
the lidar produces non-empty range data, which the handlers publish at their rates.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Simulation constants === geometry / timing / actuation (no bare literals downstream)
TIME_STEP = 2e-3                      # physics step (s)
TIRE_STEP = 1e-3                      # tire substep (s)
SIM_END = 8.0                         # total simulated time (s)
RENDER_FPS = 50.0                     # review-video cadence

TERRAIN_LENGTH = 100.0                # terrain patch X extent (m)
TERRAIN_WIDTH = 100.0                 # terrain patch Y extent (m)
TERRAIN_TOP_Z = 0.0                   # terrain top surface height (m)

VEH_INIT_X = -10.0                    # chassis spawn X (m), drives toward +X
VEH_INIT_Y = 0.0                      # chassis spawn Y (m)
SUSPENSION_REF_HEIGHT = 0.5           # chassis-origin height above wheel-bottom at rest (HMMWV)
VEH_INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived chassis spawn Z
TIRE_RADIUS = 0.46                    # HMMWV tire radius (m), for the footprint assert
ZTOL = 0.10                           # allowed wheel-bottom clearance vs terrain top

# Visualization box used as a static lidar target, placed ahead of the vehicle.
BOX_SIZE = 1.0                        # cube edge length (m)
BOX_POS = chrono.ChVector3d(5.0, 0.0, TERRAIN_TOP_Z + BOX_SIZE / 2.0)

# Camera viewpoint requested for this scene.
CAM_EYE = chrono.ChVector3d(-5.0, 2.5, 1.5)

# Derived constants (precomputed once — never recomputed in the hot loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
LIDAR_UPDATE_RATE = 10.0              # lidar scan rate (Hz)
LIDAR_W = 360                         # horizontal samples
LIDAR_V = 16                          # vertical channels
LIDAR_HFOV = 2.0 * math.pi            # full 360-degree horizontal field of view
LIDAR_MAX_VERT = 0.2618               # +15 deg upper vertical bound (rad)
LIDAR_MIN_VERT = -0.2618              # -15 deg lower vertical bound (rad)
LIDAR_MAX_DIST = 100.0                # max range (m)


# === ROS-shaped publishing layer (reconstructed in plain Python) ===
# No pychrono.ros module exists in this build; these classes mirror Chrono's
# ChROSHandler / ChROSManager contract (rate-gated Update -> Tick, a manager that
# ticks every handler each step) using only plain Python.
class ChROSHandler:
    """Rate-gated handler base: Update(time) fires Tick(time) at the set rate."""

    def __init__(self, update_rate, topic):
        self.update_rate = float(update_rate)          # publish rate (Hz)
        self.topic = topic                             # ROS-style topic name
        self._period = 1.0 / self.update_rate if self.update_rate > 0 else 0.0
        self._next_time = 0.0                          # next scheduled tick time
        self.last_message = None                       # most recent published payload

    def Update(self, time):
        # Fire only once the publish period has elapsed (rate gating).
        if time + 1e-9 >= self._next_time:
            self._next_time += self._period
            self.Tick(time)

    def Tick(self, time):
        raise NotImplementedError


class VehicleStatePublisher(ChROSHandler):
    """Publishes chassis world pose + forward speed (odometry-style topic)."""

    def __init__(self, update_rate, vehicle):
        super().__init__(update_rate, "/vehicle/state")
        self._vehicle = vehicle                        # cache: handle reused every tick

    def Tick(self, time):
        pos = self._vehicle.GetPos()
        self.last_message = {
            "time": time,
            "x": pos.x, "y": pos.y, "z": pos.z,
            "speed": self._vehicle.GetSpeed(),
        }


class DriverInputsHandler(ChROSHandler):
    """Subscribes-to / publishes the active driver inputs (steering/throttle/brake)."""

    def __init__(self, update_rate, driver):
        super().__init__(update_rate, "/vehicle/driver_inputs")
        self._driver = driver                          # cache: handle reused every tick

    def Tick(self, time):
        di = self._driver.GetInputs()
        self.last_message = {
            "time": time,
            "steering": di.m_steering,
            "throttle": di.m_throttle,
            "braking": di.m_braking,
        }


class LidarHandler(ChROSHandler):
    """Publishes the most-recent lidar range buffer as a ROS-style scan message."""

    def __init__(self, update_rate, lidar):
        super().__init__(update_rate, "/lidar/scan")
        self._lidar = lidar                            # cache: handle reused every tick

    def Tick(self, time):
        buf = self._lidar.GetMostRecentDIBuffer()      # may be empty before first scan
        n = 0
        if buf.HasData():                              # guard: only read a filled buffer
            n = int(buf.Width) * int(buf.Height)
        self.last_message = {"time": time, "topic": self.topic, "num_points": n}


class ChROSPythonManager:
    """Owns the handlers and ticks each one every physics step (rate gating internal)."""

    def __init__(self):
        self._handlers = []

    def RegisterHandler(self, handler):
        self._handlers.append(handler)

    def Update(self, time):
        for handler in self._handlers:
            handler.Update(time)


# === Vehicle (HMMWV_Full wrapper owns its ChSystemSMC) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT)
)
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)     # grippy tire so the vehicle actually drives
hmmwv.SetTireStepSize(TIRE_STEP)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                        # ChSystemSMC owned by the wrapper
# Contact scene -> Bullet collision is REQUIRED (terrain + tire contacts).
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
veh_obj = hmmwv.GetVehicle()                      # cache: vehicle handle reused below
chassis = hmmwv.GetChassisBody()                  # main chassis rigid body
# wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
# links created inside the wrapper; terrain: RigidTerrain patch body added below.

# === Footprint assert (vehicle must start ON the terrain, not through it) ===
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Terrain === flat rigid patch under the vehicle (SMC contact material)
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Lidar target box === fixed visualization box ahead of the vehicle
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.7)
box_mat.SetRestitution(0.0)
box_mat.SetYoungModulus(2e7)
target_box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, 1000.0, True, True, box_mat)
target_box.SetPos(BOX_POS)
target_box.SetFixed(True)
target_box.SetName("lidar_target_box")
system.AddBody(target_box)

# === Driver === open-loop schedule: brief settle, then accelerate forward
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(1.0, 0.0, 0.7, 0.0, 0.0),
    veh.DataDriverEntry(SIM_END, 0.0, 0.7, 0.0, 0.0),
])
driver = veh.ChDataDriver(veh_obj, driver_data)
driver.Initialize()

# === Sensors === lidar on the chassis, managed by a ChSensorManager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(20, 20, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

lidar = sens.ChLidarSensor(
    chassis,                                              # rides on the chassis body
    LIDAR_UPDATE_RATE,
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 1.0), chrono.QUNIT),  # 1 m above chassis origin
    LIDAR_W, LIDAR_V,
    LIDAR_HFOV, LIDAR_MAX_VERT, LIDAR_MIN_VERT, LIDAR_MAX_DIST,
)
lidar.SetName("chassis_lidar")
lidar.PushFilter(sens.ChFilterDIAccess())                 # range/intensity buffer access
lidar.PushFilter(sens.ChFilterPCfromDepth())              # depth -> point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(960, 600, 1.0))  # live point-cloud preview
lidar.PushFilter(sens.ChFilterSavePtCloud("cam/lidar/"))  # point-cloud frames -> mp4 later
manager.AddSensor(lidar)

# === ROS-shaped publishers (plain-Python reconstruction) ===
ros_manager = ChROSPythonManager()
ros_manager.RegisterHandler(VehicleStatePublisher(25.0, veh_obj))
ros_manager.RegisterHandler(DriverInputsHandler(25.0, driver))
ros_manager.RegisterHandler(LidarHandler(LIDAR_UPDATE_RATE, lidar))

# === Visualization === full vehicle Irrlicht scene: window + sky + camera + lights + terrain
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with ROS-shaped publishers + lidar")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()                                          # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()                                           # standard outdoor sky backdrop
vis.AddCamera(CAM_EYE, BOX_POS)                           # requested fixed viewpoint
vis.AddTypicalLights()                                    # standard lighting
vis.AddGrid(1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z + 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))                # ground reference grid
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Main loop === drive forward; pump sensors + ROS handlers; render at cadence

try:

    frame = 0
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()


            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)          # advances the wrapper-owned system
            vis.Advance(TIME_STEP)

            manager.Update()                  # pump the lidar each physics step
            ros_manager.Update(time)          # tick ROS-shaped handlers (rate-gated)

            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
