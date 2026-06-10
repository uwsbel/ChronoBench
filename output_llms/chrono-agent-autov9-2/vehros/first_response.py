"""HMMWV wheeled vehicle on rigid terrain, driven open-loop, with a ROS-shaped
communication layer reconstructed in pure Python.

Model: a full-model HMMWV (catalog `veh.HMMWV_Full`, SMC contact, SHAFTS engine,
automatic-shafts transmission, AWD driveline, Pitman-arm steering, TMEASY tires)
spawned on a flat `veh.RigidTerrain` patch with defined friction and restitution.
The wrapper owns its `ChSystemSMC`; terrain, driver, and visualization attach to it.

System type: SMC (the HMMWV wrapper builds a ChSystemSMC for SHAFTS/TMEASY).
Main bodies: chassis + 4 spindles/wheels (created inside the wrapper), the rigid
terrain patch body.

ROS layer: this PyChrono build ships no `pychrono.ros` module, so the ROS message
flow is reconstructed in plain Python the way the C++ Chrono::ROS handlers behave:
a rate-gated `RosHandler` base (Update -> Tick when the publish period elapses), a
clock-synchronization handler, a vehicle-state publisher handler (chassis pose +
twist + speed), and a driver-inputs handler that feeds steering/throttle/braking
into the vehicle exactly as `ChROSDriverInputsHandler` would. A `RosPythonManager`
registers the handlers and is updated once per physics step to publish data.

Expected behavior: the HMMWV accelerates forward from rest under throttle, the
ROS-shaped manager publishes chassis state each tick and applies the driver-input
message to the vehicle, and the chassis translates several meters along +X.
"""

import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants === geometry / physics / control knobs (no bare literals downstream)
TIME_STEP = 2e-3                         # integrator + tire sub-step
TIRE_STEP = 1e-3                         # TMEASY tire step (<= TIME_STEP)
SIM_END = 8.0                            # seconds of simulated driving
RENDER_FPS = 30.0                        # review-video cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

TERRAIN_LENGTH = 100.0                   # X extent of rigid patch (m)
TERRAIN_WIDTH = 100.0                    # Y extent of rigid patch (m)
TERRAIN_TOP_Z = 0.0                      # patch top plane height (m)
TERRAIN_FRICTION = 0.8                   # prompt: defined terrain friction
TERRAIN_RESTITUTION = 0.01               # prompt: defined terrain restitution
TERRAIN_YOUNG = 2e7                      # SMC patch stiffness (Pa)

SUSPENSION_REF_HEIGHT = 0.5              # HMMWV chassis origin above wheel-bottom at rest
TIRE_RADIUS = 0.46                       # HMMWV tire radius (m), for footprint assert
ZTOL = 0.10                              # allowed wheel-bottom clearance vs support top

VEH_INIT_X = 0.0                         # spawn X (m)
VEH_INIT_Y = 0.0                         # spawn Y (m)
VEH_INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT     # derived chassis-origin height

ROS_STATE_RATE = 25.0                    # vehicle-state publish rate (Hz)
ROS_INPUT_RATE = 25.0                    # driver-input message rate (Hz)


# === ROS-shaped communication layer (pure Python; no pychrono.ros module exists) ===
# Mirrors Chrono::ROS: a rate-gated handler base (Update gates on the publish period,
# then calls Tick), concrete clock / vehicle-state / driver-input handlers, and a
# manager that registers handlers and ticks them once per step to publish/consume.
class RosHandler:
    """Base handler: Update() is rate-gated; Tick() does the actual publish/consume."""

    def __init__(self, update_rate, topic):
        self._period = 1.0 / update_rate if update_rate > 0 else 0.0   # publish period (s)
        self._topic = topic
        self._last_time = -float("inf")

    def Initialize(self):
        return True

    def Update(self, time):
        # Rate gate: only Tick once a full publish period has elapsed.
        if time - self._last_time + 1e-12 >= self._period:
            self._last_time = time
            self.Tick(time)

    def Tick(self, time):
        raise NotImplementedError


class RosClockHandler(RosHandler):
    """Publishes the simulation clock on /clock (clock synchronization)."""

    def __init__(self):
        super().__init__(0.0, "/clock")   # rate 0 -> publish every step
        self.sim_time = 0.0

    def Tick(self, time):
        self.sim_time = time              # stand-in for rosgraph_msgs/Clock publish


class RosVehicleStateHandler(RosHandler):
    """Publishes chassis pose + twist + speed (vehicle-state topic)."""

    def __init__(self, vehicle_obj, update_rate, topic="/vehicle/state"):
        super().__init__(update_rate, topic)
        self._vehicle = vehicle_obj       # cache: vehicle handle reused every tick
        self._chassis = vehicle_obj.GetChassisBody()   # cache: chassis fetched once
        self.last_msg = None

    def Tick(self, time):
        pos = self._chassis.GetPos()
        rot = self._chassis.GetRot()
        vel = self._chassis.GetPosDt()
        # Message payload mirrors a nav_msgs/Odometry-style struct.
        self.last_msg = {
            "stamp": time,
            "x": pos.x, "y": pos.y, "z": pos.z,
            "qw": rot.e0, "qx": rot.e1, "qy": rot.e2, "qz": rot.e3,
            "vx": vel.x, "vy": vel.y, "vz": vel.z,
            "speed": self._vehicle.GetSpeed(),
        }


class RosDriverInputsHandler(RosHandler):
    """Consumes a driver-inputs topic and applies it to the vehicle's driver.

    Mirrors ChROSDriverInputsHandler: an external publisher would set the target
    steering/throttle/braking; here a scripted open-loop schedule provides them,
    and Tick latches them into the ChDriver via its Set* methods.
    """

    def __init__(self, driver_obj, update_rate, topic="/vehicle/driver_inputs"):
        super().__init__(update_rate, topic)
        self._driver = driver_obj         # cache: driver handle reused every tick
        self.last_cmd = (0.0, 0.0, 0.0)

    def message_for(self, time):
        # Open-loop driver-input "message": brake briefly, then steady throttle.
        if time < 0.5:
            steering, throttle, braking = 0.0, 0.0, 1.0
        else:
            steering = 0.25 * math.sin(0.4 * time)
            throttle, braking = 0.7, 0.0
        return steering, throttle, braking

    def Tick(self, time):
        steering, throttle, braking = self.message_for(time)
        self._driver.SetSteering(steering)
        self._driver.SetThrottle(throttle)
        self._driver.SetBraking(braking)
        self.last_cmd = (steering, throttle, braking)


class RosPythonManager:
    """Registers ROS-shaped handlers and updates them each step (data publishing)."""

    def __init__(self):
        self._handlers = []

    def RegisterHandler(self, handler):
        self._handlers.append(handler)

    def Initialize(self):
        for h in self._handlers:
            h.Initialize()

    def Update(self, time):
        # Tick every registered handler; rate gating happens inside each handler.
        for h in self._handlers:
            h.Update(time)


# === Scripted driver === ChDriver subclass; inputs are driven by the ROS handler
class RosDriver(veh.ChDriver):
    """Driver whose Set* state is written by RosDriverInputsHandler each tick."""

    def __init__(self, vehicle_obj):
        super().__init__(vehicle_obj)


# === Vehicle (catalog HMMWV_Full wrapper; owns its ChSystemSMC) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)        # prompt: contact method
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT)
)
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)           # prompt: engine type
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)               # prompt: tire model
hmmwv.SetTireStepSize(TIRE_STEP)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                # ChSystemSMC owned by the wrapper
vehicle_obj = hmmwv.GetVehicle()          # cache: ChWheeledVehicle reused every step
chassis = hmmwv.GetChassisBody()          # cache: main chassis rigid body, reused
# wheels/spindles: vehicle_obj.GetSpindlePos(axle, side); joints (suspension +
# steering links) are created inside the wrapper; terrain patch body added below.

# Collision system REQUIRED: vehicle + terrain is a contact scene. Set it on the
# wrapper-owned system after Initialize (do not create a second system).
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === flat rigid patch with defined friction + restitution
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch_mat.SetYoungModulus(TERRAIN_YOUNG)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Footprint assert: wheels must rest on (not through) the terrain top.
spindle_world = [
    vehicle_obj.GetSpindlePos(axle, side)
    for axle in range(vehicle_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === scripted ChDriver driven through the ROS-shaped input handler
driver = RosDriver(vehicle_obj)
driver.Initialize()

# === ROS-shaped manager === register clock + state-publisher + driver-input handlers
ros_manager = RosPythonManager()
clock_handler = RosClockHandler()
state_handler = RosVehicleStateHandler(vehicle_obj, ROS_STATE_RATE)
inputs_handler = RosDriverInputsHandler(driver, ROS_INPUT_RATE)
ros_manager.RegisterHandler(clock_handler)
ros_manager.RegisterHandler(inputs_handler)
ros_manager.RegisterHandler(state_handler)
ros_manager.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + sky + chase camera + lights + grid
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with ROS-shaped communication")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 60, 60,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z + 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
vis.AttachVehicle(vehicle_obj)
vis.AttachDriver(driver)

# === Main loop === advance driver/terrain/vehicle/vis + tick the ROS manager each step

try:

    frame = 0
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()

            # ROS-shaped publish/consume: ticks the input handler (drives the driver)
            # and the state publisher before stepping the dynamics.
            ros_manager.Update(sim_time)

            driver_inputs = driver.GetInputs()

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)        # advances the wrapper-owned ChSystemSMC
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:    # solver divergence / invalid vehicle state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === close writers, assemble review video + plot, drop frame dirs
