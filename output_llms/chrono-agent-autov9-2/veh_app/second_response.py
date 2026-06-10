"""HMMWV wheeled vehicle on flat rigid terrain with an onboard 360 lidar.

Models a full HMMWV (ChSystemSMC owned by the veh.HMMWV_Full wrapper) spawned at
world (0, -5, 0.4) on a flat RigidTerrain patch. Two static blue props sit at the
origin: a 1x1x1 box (center z=0.5) and a r=0.5, h=1 cylinder (center z=1.5). A
roof-mounted ChLidarSensor (offset z=2 on the chassis) sweeps a full 360 deg
horizontal field with 800 horizontal samples and 300 vertical channels, feeding
depth+intensity (DI) and XYZI point-cloud data filters. An onboard chase
camera sensor records the scene. A scripted driver holds steering=0.5 and
throttle=0.2, so the vehicle accelerates while turning, driving a curved path that
sweeps the lidar across the two props.

System type: SMC (HMMWV wrapper default). Expected behavior: the vehicle pulls
away from its spawn and curves left under constant steering+throttle while the
onboard lidar continuously scans the surrounding scene including the box and
cylinder near the origin.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics) ===
TIME_STEP = 2e-3                       # integration step (s)
TIRE_STEP = 1e-3                       # tire substep (s)
SIM_END = 10.0                         # simulation duration (s)
RENDER_FPS = 50.0                      # review-video frame rate

TERRAIN_LENGTH = 100.0                 # terrain patch X size (m)
TERRAIN_WIDTH = 100.0                  # terrain patch Y size (m)
TERRAIN_TOP_Z = 0.0                    # flat terrain top plane (m)

VEH_INIT_LOC = chrono.ChVector3d(0, -5, 0.4)   # chassis-origin spawn (world)
VEH_INIT_ROT = chrono.QUNIT                     # facing +X
TIRE_RADIUS = 0.464                    # HMMWV tire radius (m), for footprint assert

# Static blue props at the origin.
BOX_SIZE = chrono.ChVector3d(1.0, 1.0, 1.0)     # full extents (m)
BOX_POS = chrono.ChVector3d(0, 0, 0.5)
CYL_RADIUS = 0.5
CYL_HEIGHT = 1.0
CYL_POS = chrono.ChVector3d(0, 0, 1.5)
BLUE_TEXTURE = chrono.GetChronoDataFile("textures/blue.png")

# Onboard lidar specification.
LIDAR_OFFSET = chrono.ChVector3d(0.0, 0, 2)     # roof mount above chassis origin
LIDAR_HSAMPLES = 800                            # horizontal samples
LIDAR_VCHANNELS = 300                           # vertical channels
LIDAR_HFOV = 2 * chrono.CH_PI                   # 360 deg horizontal field of view
LIDAR_MAX_VERT = chrono.CH_PI / 12              # max vertical angle
LIDAR_MIN_VERT = -chrono.CH_PI / 6              # min vertical angle
LIDAR_MAX_RANGE = 100.0                         # max range (m)
LIDAR_SAMPLE_RADIUS = 2
LIDAR_DIVERGENCE = 0.003                        # beam divergence angle (rad)
LIDAR_UPDATE_RATE = 5.0                         # modest lidar update rate (Hz)

# Onboard chase camera (records the review video without GL-readback contention).
CAM_OFFSET = chrono.ChVector3d(-8.0, 0.0, 3.0)  # behind + above the chassis (local)
CAM_TARGET = chrono.ChVector3d(2.0, 0.0, 0.5)   # look ahead at the chassis nose
CAM_WIDTH = 1280
CAM_HEIGHT = 720
CAM_FOV = 1.4                                   # horizontal FOV (rad)
CAM_UPDATE_RATE = 30.0                          # camera frame rate (Hz)

# Scripted driver inputs (held constant through the run).
DRIVE_STEERING = 0.5
DRIVE_THROTTLE = 0.2

# Derived constants (precomputed once).
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))           # precomputed once

# === Vehicle (HMMWV_Full wrapper owns its ChSystemSMC) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(VEH_INIT_LOC, VEH_INIT_ROT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)            # grippy tire for rigid road
hmmwv.SetTireStepSize(TIRE_STEP)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()               # cache: ChWheeledVehicle handle, reused below
# spindles/wheels: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
# links created inside the wrapper; terrain patch body added below.

# Collision system is REQUIRED for this contact scene (tires vs terrain + props).
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Footprint assertion (wheels rest on the terrain, not through it) ===
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - 0.10, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise VEH_INIT_LOC.z"
)

# === Terrain (flat rigid patch under the vehicle) ===
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Static props (blue box + blue cylinder at the origin) ===
prop_mat = chrono.ChContactMaterialSMC()
prop_mat.SetFriction(0.8)
prop_mat.SetRestitution(0.0)
prop_mat.SetYoungModulus(2e7)

box = chrono.ChBodyEasyBox(BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, 1000, True, True, prop_mat)
box.SetPos(BOX_POS)
box.SetFixed(True)
box.SetName("blue_box")
box.GetVisualShape(0).SetTexture(BLUE_TEXTURE)
system.AddBody(box)

cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, CYL_RADIUS, CYL_HEIGHT, 1000, True, True, prop_mat)
cylinder.SetPos(CYL_POS)
cylinder.SetFixed(True)
cylinder.SetName("blue_cylinder")
cylinder.GetVisualShape(0).SetTexture(BLUE_TEXTURE)
system.AddBody(cylinder)

# === Sensors (onboard 360 lidar on the chassis) ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1, 1, 1), 5000.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

lidar = sens.ChLidarSensor(
    chassis,                                            # rides on the chassis body
    LIDAR_UPDATE_RATE,
    chrono.ChFramed(LIDAR_OFFSET, chrono.QUNIT),        # roof offset pose
    LIDAR_HSAMPLES,
    LIDAR_VCHANNELS,
    LIDAR_HFOV,
    LIDAR_MAX_VERT,
    LIDAR_MIN_VERT,
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    LIDAR_SAMPLE_RADIUS,
    LIDAR_DIVERGENCE,
    LIDAR_DIVERGENCE,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("onboard_lidar")
lidar.PushFilter(sens.ChFilterDIAccess())               # depth + intensity data buffer
lidar.PushFilter(sens.ChFilterPCfromDepth())            # convert depth -> point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())             # XYZI point-cloud data buffer
manager.AddSensor(lidar)

# Chase camera sensor (OptiX) rides on the chassis and records the review frames.
cam_forward = (CAM_TARGET - CAM_OFFSET).GetNormalized()                 # precomputed once
cam_quat = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), cam_forward)
camera = sens.ChCameraSensor(
    chassis,                                            # follows the vehicle
    CAM_UPDATE_RATE,
    chrono.ChFramed(CAM_OFFSET, cam_quat),              # local chase offset + look-at
    CAM_WIDTH, CAM_HEIGHT, CAM_FOV,
)
camera.SetName("chase_camera")
camera.PushFilter(sens.ChFilterSave("cam/chase/"))      # PNG frames -> review video
camera.PushFilter(sens.ChFilterRGBA8Access())           # frame-buffer access
manager.AddSensor(camera)

# === Driver (scripted constant steering + throttle) ===
class ConstantDriver(veh.ChDriver):
    """Holds a fixed steering and throttle for the whole run."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetSteering(DRIVE_STEERING)
        self.SetThrottle(DRIVE_THROTTLE)
        self.SetBraking(0.0)

driver = ConstantDriver(veh_obj)
driver.Initialize()

# === Visualization === full vehicle Irrlicht scene: window + sky + camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV with onboard lidar")
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 60, 60,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)


# === Main loop === advance the full vehicle subsystem stack; lidar scans every step
try:
    while vis.Run() and system.GetChTime() < SIM_END:
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
            manager.Update()                 # pump the lidar every physics step
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)          # advances the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
