"""FED-Alpha (FEDA) ISO double lane-change with a path-follower, cruise-control driver.

Models the FEDA catalog wheeled vehicle on a flat rigid terrain patch, driven
autonomously by a ChPathFollowerDriver that tracks the ISO standard double
lane-change path at a constant target speed (cruise control). The interactive
keyboard driver is replaced by this autonomous path follower.

System type: NSC (ChContactMethod_NSC) with the Bullet collision system.
Main bodies: FEDA chassis + four wheels/spindles, and a flat RigidTerrain patch.
Expected behavior: the vehicle accelerates to the target speed and steers through
the double lane-change weave, staying on the 200 m terrain patch.
"""

import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry, controller gains, and timing as named constants
TIME_STEP = 1e-3
SIM_END = 20.0
RENDER_FPS = 50.0

TARGET_SPEED = 10.0                       # cruise-control target speed (m/s)
STEER_LOOK_AHEAD = 5.0                    # path-follower look-ahead distance (m)
STEER_KP, STEER_KI, STEER_KD = 0.8, 0.0, 0.0   # steering controller gains
SPEED_KP, SPEED_KI, SPEED_KD = 0.4, 0.0, 0.0   # speed controller gains

VEHICLE_INIT = chrono.ChVector3d(-50.0, 0.0, 0.5)   # moved so the maneuver fits the patch
TERRAIN_LENGTH = 200.0                   # lengthened so the maneuver fits the patch
TERRAIN_WIDTH = 10.0

# Double-lane-change path geometry (ISO standard maneuver).
DLC_LENGTH = 13.5
DLC_WIDTH = 4.0
DLC_OFFSET = 11.0
DLC_TOTAL_LENGTH = 50.0

# === Data paths === locate bundled Chrono + vehicle assets (truth-faithful)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle === FEDA catalog wrapper on rigid terrain (NSC), owns its ChSystem
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)        # NSC for rigid terrain
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)                              # fixed chassis would never move
feda.SetInitPosition(chrono.ChCoordsysd(VEHICLE_INIT, chrono.QUNIT))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(TIME_STEP)
feda.Initialize()

# System + bodies created internally by the FEDA wrapper.
system = feda.GetSystem()                                # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = feda.GetChassisBody()                          # cache: main chassis body, reused below
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain === flat rigid patch lengthened to 200 m to contain the maneuver
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver === path-follower cruise-control on the ISO double lane-change path
path = veh.DoubleLaneChangePath(
    VEHICLE_INIT, DLC_LENGTH, DLC_WIDTH, DLC_OFFSET, DLC_TOTAL_LENGTH, True
)
driver = veh.ChPathFollowerDriver(
    feda.GetVehicle(), path, "double_lane_change", TARGET_SPEED
)
steer_ctrl = driver.GetSteeringController()              # cache: steering controller
steer_ctrl.SetLookAheadDistance(STEER_LOOK_AHEAD)
steer_ctrl.SetGains(STEER_KP, STEER_KI, STEER_KD)
driver.GetSpeedController().SetGains(SPEED_KP, SPEED_KI, SPEED_KD)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA - ISO Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(feda.GetVehicle())

# === Main loop === advance driver/terrain/vehicle/vis in lock-step (real-time)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()


frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            driver.Synchronize(time)
            terrain.Synchronize(time)
            feda.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            feda.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)

            realtime_timer.Spin(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:          # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
