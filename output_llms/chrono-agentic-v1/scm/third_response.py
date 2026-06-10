"""
SCM Off-Road Vehicle Simulation with Random Obstacle Boxes and Camera Sensor.

Models an HMMWV full-model vehicle driving on SCM (Bekker-Wong) soft-soil terrain.
Random ChBodyEasyBox obstacles are placed in the scene ensuring no box overlaps the
vehicle spawn region. A sensor manager is created with multiple point lights and a
ChCameraSensor attached to the vehicle chassis. The camera visualizes its feed live
via ChFilterVisualize and saves frames via ChFilterSave + ChFilterRGBA8Access.

System: ChSystemNSC (vehicle-owned), with Bullet collision.
Main bodies: HMMWV chassis + axles, SCM terrain grid, random box obstacles.
Expected: vehicle drives forward on deformable soil, ruts form, camera tracks chassis.
"""

# === Imports ===
import math
import os
import random
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# Anchor bundled vehicle asset subtree
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named Constants ===
# Simulation timing
TIME_STEP = 1e-3                # physics step size (s)
SIM_END = 20.0                  # total simulation duration (s)
RENDER_FPS = 50.0               # Irrlicht window render rate (Hz)
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
render_steps = math.ceil(RENDER_STEP_SIZE / TIME_STEP)  # precomputed once

# Terrain
TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 120.0
TERRAIN_RESOLUTION = 0.1       # SCM grid resolution (m)

# Vehicle spawn
INIT_X = 0.0
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5    # chassis origin above wheel-bottom at rest (m)
TERRAIN_Z = 0.0                # SCM rest plane
INIT_Z = TERRAIN_Z + SUSPENSION_REF_HEIGHT
INIT_POS = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.QuatFromAngleZ(0.0)

# Random boxes
NUM_BOXES = 20
BOX_MIN_SIZE = 0.3
BOX_MAX_SIZE = 1.0
BOX_DENSITY = 500.0
# Safe zone around vehicle spawn (no box within this radius in XY)
SAFE_RADIUS = 6.0
# Box scatter area (keep boxes on terrain)
SCATTER_HALF = 40.0

# Sensor camera
CAM_UPDATE_RATE = 30           # Hz — physical rate, not 1/dt
CAM_WIDTH = 1280
CAM_HEIGHT = 720
CAM_FOV = 1.408                # horizontal FOV (rad)

# === Vehicle Setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)      # SCM scene needs SMC
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                            # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)             # TMEASY required for SCM traction
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === System & Bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()          # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain
chassis = hmmwv.GetChassisBody()    # cache: main chassis rigid body — fetched once, reused
# Axles/spindles/tires created inside wrapper; terrain attached below.

# Verify wheel-bottom clearance after Initialize
TIRE_RADIUS = 0.33
ZTOL = 0.05
veh_obj = hmmwv.GetVehicle()
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle_idx, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_Z - ZTOL, (
    f"Vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain z={TERRAIN_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_Z - wheel_bottom_z:.3f} m"
)

# Visualization types — set after Initialize
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === SCM Terrain ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi — frictional modulus (Pa)
    0,      # Bekker_Kc   — cohesive modulus
    1.1,    # Bekker_n    — pressure-sinkage exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear deformation coefficient (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R     — vertical damping (Pa·s/m)
)
terrain.AddMovingPatch(
    chassis,                            # CORRECT: attach to chassis, NOT spindles
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),         # OOBB dimensions covering vehicle footprint
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)  # sinkage heatmap visualization
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RESOLUTION)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    80, 80,
)

# === Tire Collision Cylinders (REQUIRED for TMEASY on SCM) ===
# SCM ray-casts need explicit collision cylinders on each spindle for TMEASY tires.
tire_rad = veh_obj.GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w = veh_obj.GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

TIRE_FAMILY = 1
SUPPORT_FAMILY = 4

for axle in veh_obj.GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(
                tire_mat, tire_rad + 0.04, tire_w    # +0.04 ensures SCM detects sinkage
            ),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)
        # NOTE: do NOT DisallowCollisionsWith(0) — family 0 is SCM ray-cast default

system.GetCollisionSystem().BindAll()  # MANDATORY after post-init collision shape changes

# === Random Box Obstacles ===
# Place NUM_BOXES ChBodyEasyBox objects randomly; no box within SAFE_RADIUS of spawn.
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.7)
box_mat.SetRestitution(0.05)

random.seed(42)  # deterministic layout for repeatability
for i in range(NUM_BOXES):
    while True:
        bx = random.uniform(-SCATTER_HALF, SCATTER_HALF)
        by = random.uniform(-SCATTER_HALF, SCATTER_HALF)
        dist = math.sqrt((bx - INIT_X) ** 2 + (by - INIT_Y) ** 2)
        if dist >= SAFE_RADIUS:
            break
    bsize = random.uniform(BOX_MIN_SIZE, BOX_MAX_SIZE)
    box = chrono.ChBodyEasyBox(bsize, bsize, bsize, BOX_DENSITY, True, True, box_mat)
    box.SetName(f"box_{i:03d}")
    box.SetPos(chrono.ChVector3d(bx, by, TERRAIN_Z + bsize / 2.0))
    box.SetFixed(True)     # fixed obstacles on SCM surface
    box.EnableCollision(True)
    # Keep boxes in a separate collision family so they don't collide with tires via SCM
    bx_cm = box.GetCollisionModel()
    bx_cm.SetFamily(SUPPORT_FAMILY)
    bx_cm.DisallowCollisionsWith(TIRE_FAMILY)
    system.AddBody(box)

# Rebuild collision after adding boxes
system.GetCollisionSystem().BindAll()

# === Driver ===
# ChInteractiveDriverIRR — real-time keyboard/gamepad control, truth-faithful form.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM — Boxes + Camera Sensor")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0   # s to reach full steering
throttle_time = 1.0   # s to reach full throttle
braking_time = 0.3    # s to reach full braking
driver.SetSteeringDelta(RENDER_STEP_SIZE / steering_time)
driver.SetThrottleDelta(RENDER_STEP_SIZE / throttle_time)
driver.SetBrakingDelta(RENDER_STEP_SIZE / braking_time)
driver.Initialize()

# === Sensor System — Manager + Point Lights + Camera ===
manager = sens.ChSensorManager(system)

# Point lights at various positions for good illumination of the scene
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 20),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(15, 0, 20),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-15, 0, 20),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 15, 20),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Camera sensor attached to vehicle chassis, offset behind and above
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 2.5),                               # behind and above chassis
    chrono.QuatFromAngleAxis(0.25, chrono.ChVector3d(0, 1, 0)),  # tilt down slightly
)
cam = sens.ChCameraSensor(
    chassis,            # attach to REAL chassis body — follows the vehicle
    CAM_UPDATE_RATE,    # physical Hz (30), NOT 1/dt
    cam_offset_pose,
    CAM_WIDTH,
    CAM_HEIGHT,
    CAM_FOV,
)
cam.SetName("Chassis Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain: visualize live feed + save frames + host buffer access
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "Chassis Camera Feed"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/chassis_cam/"))
manager.AddSensor(cam)

# === Review-only recording setup ===


# === Main Loop ===
realtime_timer = chrono.ChRealtimeStepTimer()   # real-time pacing
step_number = 0
frame = 0                                       # Irrlicht frame counter

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        time = hmmwv.GetSystem().GetChTime()    # cached per iteration

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)        # advances wrapper-owned ChSystem — do NOT also call DoStepDynamics
        vis.Advance(TIME_STEP)

        manager.Update()                # pump sensors every physics step


        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
