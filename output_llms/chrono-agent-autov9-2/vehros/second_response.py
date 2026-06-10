"""HMMWV wheeled vehicle on flat rigid terrain, driven through a ROS-shaped
publish/subscribe control layer (NSC contact, Bullet collision).

Model
-----
- A full-model HMMWV (`veh.HMMWV_Full`) with SHAFTS engine, automatic-shafts
  transmission, all-wheel drive, Pitman-arm steering, and TMEASY tires. The
  wrapper creates and owns its `ChSystemNSC`; terrain and visualization attach
  to that same owned system.
- A flat `veh.RigidTerrain` patch with an NSC contact material provides the
  driving surface; the wheels rest on it at spawn (asserted from the spindle
  world positions after Initialize).

ROS-shaped control layer (plain Python, no external middleware)
---------------------------------------------------------------
PyChrono 9.0.1 ships no ROS Python module, so the ROS data flow is reconstructed
in plain Python with the same shapes the C++ Chrono::ROS bridge uses:
- a `ChROSHandler` base whose `Tick(time)` is rate-gated so an `Update(time)`
  body runs at a fixed publish rate (the standard ROS handler cadence);
- a vehicle-state publisher handler that reads chassis pose/speed each tick and
  appends to an in-memory "/vehicle/state" topic buffer (what a real publisher
  would serialize onto the wire);
- a driver-inputs handler that holds the latest "/vehicle/driver_inputs"
  command (throttle/steering/braking) and applies it to the vehicle's
  `veh.DriverInputs` struct, mirroring a subscribed command topic;
- a `ChROSPythonManager` that registers the handlers and is ticked once per
  physics step, so every handler advances on its own rate gate.

Expected behavior
------------------
The driver-inputs handler commands a short brake-then-accelerate-and-steer
profile; the HMMWV accelerates forward from rest and turns, while the
state-publisher handler streams its pose/speed. The chassis X displacement at
the end must be clearly positive (the vehicle moves).
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Named constants === geometry / physics / control, no bare literals downstream
TIME_STEP = 1.0e-3                  # integration step (s)
SIM_END = 8.0                       # simulated duration (s)
RENDER_FPS = 50.0                   # review-video frame rate
ROS_PUBLISH_RATE = 25.0             # Hz — vehicle-state publish cadence
ROS_CONTROL_RATE = 50.0             # Hz — driver-input command cadence

TERRAIN_LENGTH = 100.0              # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0               # rigid patch Y extent (m)
TERRAIN_TOP_Z = 0.0                 # patch top surface height (m)

SUSPENSION_REF_HEIGHT = 0.5         # chassis origin above wheel-bottom at rest (m)
VEH_INIT_X = 0.0                    # spawn X from the scene (m)
VEH_INIT_Y = 0.0                    # spawn Y from the scene (m)
VEH_INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived chassis-origin height
ZTOL = 0.05                         # allowed wheel-bottom clearance/overlap vs terrain

# Open-loop command profile published on the driver-inputs topic.
BRAKE_RELEASE_T = 0.5               # brake fully until this time, then drive (s)
CRUISE_THROTTLE = 0.7               # throttle once moving
STEER_AMPLITUDE = 0.3               # steady steering after the straight launch

# === ROS-shaped control layer === handler base + publisher + subscriber + manager
class ChROSHandler:
    """ROS handler base: a rate-gated Tick(time) that calls Update(time).

    This mirrors the C++ Chrono::ROS handler contract — the manager calls
    Tick every physics step and the handler only does work when its fixed
    publish/update period has elapsed.
    """

    def __init__(self, update_rate):
        self._period = 1.0 / update_rate          # cache: fixed handler period (s)
        self._next_time = 0.0                      # next wall time this handler fires

    def Tick(self, time):
        if time + 1e-12 >= self._next_time:
            self.Update(time)
            self._next_time += self._period

    def Update(self, time):
        raise NotImplementedError


class VehicleStatePublisher(ChROSHandler):
    """Publishes chassis pose + speed onto the '/vehicle/state' topic buffer."""

    def __init__(self, chassis_body, vehicle, update_rate):
        super().__init__(update_rate)
        self._chassis = chassis_body               # cache: chassis body, reused every tick
        self._vehicle = vehicle                    # cache: ChWheeledVehicle, reused every tick
        self.messages = []                         # in-memory published-topic buffer

    def Update(self, time):
        pos = self._chassis.GetPos()
        speed = self._vehicle.GetSpeed()
        self.messages.append((time, pos.x, pos.y, pos.z, speed))


class DriverInputsHandler(ChROSHandler):
    """Holds the latest '/vehicle/driver_inputs' command and applies it.

    Acts as the subscriber side: a scripted publisher (the open-loop profile)
    sets the command, and Update() copies it onto the live DriverInputs struct
    the vehicle reads — exactly what a subscribed command-topic callback does.
    """

    def __init__(self, driver_inputs, update_rate):
        super().__init__(update_rate)
        self._inputs = driver_inputs               # cache: shared DriverInputs struct
        self._cmd_throttle = 0.0
        self._cmd_steering = 0.0
        self._cmd_braking = 0.0

    def Publish(self, throttle, steering, braking):
        # Scripted command publisher writing onto the driver-inputs topic.
        self._cmd_throttle = throttle
        self._cmd_steering = steering
        self._cmd_braking = braking

    def Update(self, time):
        self._inputs.m_throttle = self._cmd_throttle
        self._inputs.m_steering = self._cmd_steering
        self._inputs.m_braking = self._cmd_braking


class ChROSPythonManager:
    """Registers ROS handlers and ticks each one every physics step."""

    def __init__(self):
        self._handlers = []

    def RegisterHandler(self, handler):
        self._handlers.append(handler)

    def Update(self, time):
        for handler in self._handlers:
            handler.Tick(time)


def open_loop_command(time):
    """Scripted profile the driver-inputs publisher sends each control tick."""
    if time < BRAKE_RELEASE_T:
        return 0.0, 0.0, 1.0                       # hold on the brake at the start
    steering = STEER_AMPLITUDE * math.sin(0.4 * (time - BRAKE_RELEASE_T))
    return CRUISE_THROTTLE, steering, 0.0          # accelerate forward and steer


# === Vehicle === full HMMWV wrapper owns its ChSystemNSC; configure then Initialize
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT)
)
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)         # prompt: TMEASY tire on rigid road
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                           # ChSystemNSC owned by the wrapper
# Collision REQUIRED — vehicle wheels contact the rigid terrain (Bullet narrow-phase).
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
veh_obj = hmmwv.GetVehicle()                         # cache: ChWheeledVehicle, reused every step
chassis = hmmwv.GetChassisBody()                     # cache: chassis rigid body, reused every step
# spindles/wheels: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
# links created inside the wrapper; terrain: RigidTerrain patch body added below.

# === Terrain === flat rigid patch with an NSC contact material under the wheels
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Footprint assert === wheels must start on (not through) the rigid terrain
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
TIRE_RADIUS = veh_obj.GetAxles()[0].GetWheels()[0].GetTire().GetRadius()
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver + ROS handlers === scripted command flows through the ROS-shaped layer
driver_inputs = veh.DriverInputs()                   # live struct the vehicle reads each step
state_pub = VehicleStatePublisher(chassis, veh_obj, ROS_PUBLISH_RATE)
inputs_handler = DriverInputsHandler(driver_inputs, ROS_CONTROL_RATE)
ros_manager = ChROSPythonManager()
ros_manager.RegisterHandler(state_pub)
ros_manager.RegisterHandler(inputs_handler)

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + terrain
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV — ROS-shaped driver bridge")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)

# === Main loop === throttled render outer loop; ROS manager + Synchronize/Advance inner
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

try:

    frame = 0
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()
            # Scripted publisher pushes the next command onto the driver-inputs topic.
            throttle, steering, braking = open_loop_command(sim_time)
            inputs_handler.Publish(throttle, steering, braking)
            # ROS manager ticks every handler (rate-gated) before the vehicle reads inputs.
            ros_manager.Update(sim_time)

            driver_inputs.m_gear = 1
            terrain.Synchronize(sim_time)
            veh_obj.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)


            terrain.Advance(TIME_STEP)
            veh_obj.Advance(TIME_STEP)         # advances the wrapper-owned ChSystem
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:    # solver divergence / invalid vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    # always report the final progress so a mid-run divergence is still visible
    print(f"simulation stopped at t={system.GetChTime():.3f}s of {SIM_END:.3f}s")
