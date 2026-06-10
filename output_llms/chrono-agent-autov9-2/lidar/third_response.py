"""ARTcar wheeled-vehicle lidar sensing simulation (PyChrono 9.0.x, Irrlicht renderer).

Models an ARTcar (small electric RC-scale wheeled vehicle) driving across a flat
rigid terrain patch under a constant-throttle data driver. Two lidar sensors are
rigidly mounted on the vehicle chassis at a forward-and-up offset pose: a 3D
multi-beam lidar and a planar 2D (single-vertical-beam) lidar. A third-person
camera sensor is also mounted on the chassis to provide a chase view. All sensors
ride the moving chassis, so their returns sweep across the ground plane as the
vehicle translates.

System type: NSC (the ARTcar wrapper owns its ChSystemNSC; contact between the
tires and the rigid terrain patch is resolved with the Bullet collision system).

Main bodies: ARTcar chassis + four wheels/spindles (created by the wrapper) and a
flat rigid terrain patch. Sensors: one 3D lidar, one 2D lidar, one RGB camera,
all attached to the chassis body.

Expected behavior: the vehicle accelerates forward in a straight line; both lidars
return a non-empty point cloud each scan (hits on the terrain ahead/around the
vehicle), and the point/return counts evolve as the chassis moves.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics) — no bare literals downstream ===
TIME_STEP = 2.0e-3                      # integration step (s)
TIRE_STEP = 1.0e-3                      # tire submodel step (s)
SIM_END = 8.0                           # simulated duration (s)
RENDER_FPS = 50.0                       # review-video frame cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

TERRAIN_LENGTH = 100.0                  # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0                   # rigid patch Y extent (m)
TERRAIN_FRICTION = 0.9                  # tire-ground friction
TERRAIN_RESTITUTION = 0.01              # tire-ground restitution

VEH_INIT_X = 0.0                        # chassis spawn X (m)
VEH_INIT_Y = 0.0                        # chassis spawn Y (m)
VEH_INIT_Z = 0.5                        # chassis-origin spawn height above terrain (m)

THROTTLE = 0.7                          # constant forward throttle (0..1)

# Lidar offset pose on the chassis: forward 1.0 m, centered, 1.0 m up.
LIDAR_OFFSET = chrono.ChVector3d(1.0, 0.0, 1.0)
LIDAR_UPDATE_RATE = 5.0                 # scans per second (Hz)
LIDAR_MAX_DIST = 100.0                  # max return distance (m)
LIDAR_HFOV = 2.0 * math.pi             # full 360 deg horizontal sweep (rad)

# 3D lidar vertical fan.
LIDAR3D_W = 480                         # horizontal samples
LIDAR3D_H = 32                          # vertical channels
LIDAR3D_VMAX = 0.2618                   # +15 deg upper vertical angle (rad)
LIDAR3D_VMIN = -0.2618                  # -15 deg lower vertical angle (rad)

# 2D (planar) lidar: a single scan line, tilted slightly down so the 1 m-high
# sensor's plane intersects the ground ahead and returns hits (a perfectly
# horizontal plane from 1 m up never strikes flat terrain).
LIDAR2D_W = 480                         # horizontal samples
LIDAR2D_H = 1                           # single vertical channel -> planar
LIDAR2D_VMAX = -0.0873                  # -5 deg downward scan plane (rad)
LIDAR2D_VMIN = -0.0873

CAM_W, CAM_H = 1280, 720                # third-person camera resolution
CAM_FOV = 1.408                         # horizontal FOV (rad)
CAM_OFFSET = chrono.ChVector3d(-3.0, 0.0, 1.5)  # behind + above chassis origin

# === System & vehicle (created/owned by the veh.ARTcar wrapper) ===
# The wrapper creates and owns its ChSystemNSC plus the chassis, four spindles,
# suspension/steering joints, powertrain and tires internally. We initialize it
# first, then take the owned system for terrain + sensors + visualization.
car = veh.ARTcar()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisCollisionType(veh.CollisionType_NONE)
car.SetChassisFixed(False)
car.SetInitPosition(
    chrono.ChCoordsysd(
        chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT
    )
)
car.SetTireType(veh.TireModelType_TMEASY)   # rolls on rigid terrain with slip/grip
car.SetTireStepSize(TIRE_STEP)
car.SetMaxMotorVoltageRatio(0.16)           # drive ratio for visible forward motion
car.SetStallTorque(0.5)                     # motor stall torque (N*m)
car.SetTireRollingResistance(0.06)
car.Initialize()
car.SetChassisVisualizationType(chrono.VisualizationType_MESH)
car.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
car.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
car.SetWheelVisualizationType(chrono.VisualizationType_MESH)
car.SetTireVisualizationType(chrono.VisualizationType_MESH)

system = car.GetSystem()                 # ChSystemNSC owned by the ARTcar wrapper
chassis_body = car.GetChassisBody()      # cache: main chassis rigid body, reused every step
veh_obj = car.GetVehicle()               # cache: vehicle subsystem handle, reused every step
# Spindles/wheels: veh_obj.GetAxle(i); joints: suspension + steering created in wrapper.

# === Collision system ===
# Contact scene (tires vs. rigid terrain patch) -> Bullet collision is REQUIRED.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain (flat rigid patch on the wrapper-owned system) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Footprint assert (vehicle rests on, not through, the terrain) ===
ZTOL = 0.1
spindle_world = []
for axle_i in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle_i, side))
wheel_bottom_z = min(p.z for p in spindle_world) - 0.1   # tire radius slack
terrain_top_z = 0.0
assert wheel_bottom_z >= terrain_top_z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={terrain_top_z:.3f}; raise VEH_INIT_Z"
)

# === Driver (open-loop constant-throttle data driver, headless-safe) ===
# A pre-programmed ChDataDriver replaces any human-in-the-loop input: a brief
# settle, then a steady straight-line throttle for the rest of the run.
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.6, 0.0, THROTTLE, 0.0, 0.0),
    veh.DataDriverEntry(SIM_END, 0.0, THROTTLE, 0.0, 0.0),
])
driver = veh.ChDataDriver(veh_obj, driver_data)
driver.Initialize()

# === Sensors (3D lidar + 2D lidar + third-person camera on the chassis) ===
# The sensor manager renders the sensor scene via OptiX; it needs its own lights.
# This build's ChScene has no AddDirectionalLight, so use point + ambient light.
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(20.0, 20.0, 60.0), chrono.ChColor(1.0, 1.0, 1.0), 5000.0
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-20.0, -20.0, 60.0), chrono.ChColor(1.0, 1.0, 1.0), 5000.0
)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

lidar_pose = chrono.ChFramed(LIDAR_OFFSET, chrono.QUNIT)

# 3D multi-beam lidar — DATA-ACCESS filters only (no visualize/save: those deadlock
# the Irrlicht window in this build).
lidar3d = sens.ChLidarSensor(
    chassis_body, LIDAR_UPDATE_RATE, lidar_pose,
    LIDAR3D_W, LIDAR3D_H, LIDAR_HFOV,
    LIDAR3D_VMAX, LIDAR3D_VMIN, LIDAR_MAX_DIST,
)
lidar3d.SetName("lidar_3d")
lidar3d.PushFilter(sens.ChFilterDIAccess())        # depth+intensity buffer access
lidar3d.PushFilter(sens.ChFilterPCfromDepth())     # convert depth -> point cloud
lidar3d.PushFilter(sens.ChFilterXYZIAccess())      # XYZI point-cloud access
manager.AddSensor(lidar3d)

# 2D planar lidar — single horizontal scan line, same data-access filter chain.
lidar2d = sens.ChLidarSensor(
    chassis_body, LIDAR_UPDATE_RATE, lidar_pose,
    LIDAR2D_W, LIDAR2D_H, LIDAR_HFOV,
    LIDAR2D_VMAX, LIDAR2D_VMIN, LIDAR_MAX_DIST,
)
lidar2d.SetName("lidar_2d")
lidar2d.PushFilter(sens.ChFilterDIAccess())
lidar2d.PushFilter(sens.ChFilterPCfromDepth())
lidar2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar2d)

# Third-person chase camera mounted on the chassis (RGB sensor).
cam_forward = (chrono.ChVector3d(0, 0, 0) - CAM_OFFSET).GetNormalized()  # look at chassis origin
cam_quat = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), cam_forward)
camera = sens.ChCameraSensor(
    chassis_body, LIDAR_UPDATE_RATE * 4.0,
    chrono.ChFramed(CAM_OFFSET, cam_quat),
    CAM_W, CAM_H, CAM_FOV,
)
camera.SetName("third_person_cam")
camera.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H))   # live preview window
camera.PushFilter(sens.ChFilterSave("cam/third_person/")) # PNG frames -> mp4
camera.PushFilter(sens.ChFilterRGBA8Access())             # frame-buffer access
manager.AddSensor(camera)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ARTcar lidar sensing")
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 6.0, 0.8)
vis.Initialize()                                          # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()                                           # standard outdoor sky backdrop
vis.AddTypicalLights()                                    # standard lighting
vis.AddGrid(1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))                # ground reference grid
vis.AttachVehicle(veh_obj)

# === Main loop ===

frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            # Pump all sensors so the lidars/camera see every post-step pose.
            manager.Update()


            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            car.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            car.Advance(TIME_STEP)          # advances the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Always report how far the run progressed, even if a step diverged.
    print(f"simulation ended at t={system.GetChTime():.3f} s of {SIM_END:.3f} s")

# === Post-processing ===
