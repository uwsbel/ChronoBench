"""HMMWV driving on deformable SCM (Bekker-Wong soft-soil) terrain with an onboard camera sensor.

Model
-----
- System type: NSC, owned by the ``veh.HMMWV_Full`` wrapper (SMC contact method).
- Main bodies: HMMWV chassis + four spindles/wheels (created by the wrapper), a
  deformable ``veh.SCMTerrain`` patch, and a set of randomly scattered rigid boxes
  (``ChBodyEasyBox``) placed clear of the vehicle footprint.
- Tires: TMEASY force model on SCM (rigid tires do not grip soft soil), with an
  explicit per-spindle collision cylinder so the soil ray-casts detect sinkage.
- Sensing: a ``ChSensorManager`` drives a ``ChCameraSensor`` rigidly mounted on the
  chassis (an onboard forward POV), lit by point lights + ambient light.
- Expected behavior: the vehicle accelerates forward from rest, leaves visible ruts
  in the soil, and the onboard camera records the forward driving view while the
  scattered boxes stay clear of the spawn region.
"""

# === Imports ===
import os
import math
import random

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr


# === Named constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 2e-3                       # integration step (s)
TIRE_STEP = 1e-3                       # TMEASY tire substep (s) — required on SCM
SIM_END = 8.0                          # modest run length (s)
RENDER_FPS = 30.0                      # review-video / sensor cadence (Hz)

TERRAIN_LENGTH = 40.0                  # SCM patch X extent (m)
TERRAIN_WIDTH = 40.0                   # SCM patch Y extent (m)
TERRAIN_RES = 0.1                      # SCM grid resolution (m)

SUSPENSION_REF_HEIGHT = 0.5            # HMMWV chassis origin above wheel-bottom at rest (m)
TIRE_FAMILY = 1                        # collision family for the tire cylinders
TIRE_RADIUS_PAD = 0.04                 # extra radius so SCM detects sinkage (m)

NUM_BOXES = 12                         # scattered rigid boxes
BOX_SIZE = 0.5                         # box edge length (m)
BOX_DENSITY = 50.0                     # light boxes (kg/m^3)
BOX_CLEAR_RADIUS = 6.0                 # keep boxes this far from spawn XY (m)
BOX_FIELD_HALF = 16.0                  # boxes scattered within +/- this in X/Y (m)

CAM_W, CAM_H = 1280, 720               # onboard camera resolution (px)
CAM_FOV = 1.408                        # horizontal field of view (rad)
CAM_UPDATE_RATE = 30.0                 # onboard camera update rate (Hz)

SPAWN_X, SPAWN_Y = -8.0, 0.0           # vehicle spawn XY on the patch (m)

random.seed(12345)                     # deterministic box scatter

# precomputed once — inner-batch count between rendered frames
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))            # precomputed once

# === Vehicle (HMMWV_Full wrapper owns its system + bodies) ===
# The wrapper internally creates the ChSystem (SMC), the chassis rigid body, four
# spindle bodies, and the suspension/steering joints — enumerated in named locals
# below so the essential components are visible.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(SPAWN_X, SPAWN_Y, 0.0), chrono.QUNIT)
)
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # TMEASY grips SCM; RIGID would just spin
hmmwv.SetTireStepSize(TIRE_STEP)

# Pre-sample terrain height at the spawn so the chassis rests on (not through) the soil.
# SCM rest plane is flat at z=0, so the support top is 0.0; derive the chassis Z from it.
support_top_z = 0.0
init_z = support_top_z + SUSPENSION_REF_HEIGHT
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(SPAWN_X, SPAWN_Y, init_z), chrono.QUNIT)
)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystem owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()               # cache: vehicle subsystem, reused every step
# spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering inside wrapper.

# Set the collision system explicitly so SCM ray-casts and box contacts resolve via Bullet.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Footprint assert === wheels must rest on the SCM rest plane, not sink through it
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
tire0 = veh_obj.GetAxles()[0].GetWheels()[0].GetTire()
tire_rad = tire0.GetRadius()               # cache: tire radius, reused for cylinders
tire_w = tire0.GetWidth()                  # cache: tire width, reused for cylinders
wheel_bottom_z = min(p.z for p in spindle_world) - tire_rad
assert wheel_bottom_z >= support_top_z - 0.1, (
    f"vehicle sinks into soil: wheel bottom z={wheel_bottom_z:.3f} vs support "
    f"z={support_top_z:.3f}; raise SUSPENSION_REF_HEIGHT"
)

# === Terrain === deformable SCM soft soil (firm parameters), small patch -> no active domain
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi  — frictional modulus (Pa), firm soil
    0,      # Bekker_Kc    — cohesive modulus
    1.1,    # Bekker_n     — exponent
    0,      # Mohr_cohesion
    30,     # Mohr_friction (deg)
    0.01,   # Janosi_shear (m)
    2e8,    # elastic_K (Pa/m)
    3e4,    # damping_R (Pa*s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)   # colored sinkage overlay
terrain.SetMeshWireframe(False)
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RES)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80)

# === Tire collision cylinders === TMEASY tires need explicit collision geometry for SCM
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
tire_mat.SetYoungModulus(2e7)
for axle in veh_obj.GetAxles():
    for iw in range(2):
        spindle = axle.GetWheels()[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + TIRE_RADIUS_PAD, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)   # wheels never collide with each other
# Rebuild all collision models so the new cylinders are visible to SCM ray-casts.
system.GetCollisionSystem().BindAll()

# === Scattered boxes === rejection-sample positions clear of the vehicle spawn
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.0)
box_mat.SetYoungModulus(2e7)
box_bodies = []   # cache: created once, reused for logging
for _ in range(NUM_BOXES):
    while True:                                   # rejection sampling: keep clear of vehicle
        bx = random.uniform(-BOX_FIELD_HALF, BOX_FIELD_HALF)
        by = random.uniform(-BOX_FIELD_HALF, BOX_FIELD_HALF)
        if math.hypot(bx - SPAWN_X, by - SPAWN_Y) >= BOX_CLEAR_RADIUS:
            break
    box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, BOX_DENSITY, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(bx, by, support_top_z + BOX_SIZE / 2.0))
    box.SetFixed(True)                            # static obstacles on the soft soil
    system.AddBody(box)
    box_bodies.append(box)
system.GetCollisionSystem().BindAll()

# === Sensor manager === onboard chassis camera, lit by point lights + ambient
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(20, 20, 60), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(-20, -20, 60), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# Onboard forward-looking camera, rigidly mounted on the chassis (local +X = view dir).
cam_offset = chrono.ChFramed(chrono.ChVector3d(1.6, 0.0, 1.2), chrono.QUNIT)
camera = sens.ChCameraSensor(chassis, CAM_UPDATE_RATE, cam_offset, CAM_W, CAM_H, CAM_FOV)
camera.SetName("onboard_cam")
camera.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H))   # live preview window
camera.PushFilter(sens.ChFilterSave("cam/onboard/"))      # PNG frames -> sensor video
camera.PushFilter(sens.ChFilterRGBA8Access())             # frame-buffer access
manager.AddSensor(camera)

# === Driver === scripted time-based control (no human-in-the-loop in batch runs)
class ScriptedDriver(veh.ChDriver):
    """Brief settle, then steady forward throttle with a gentle steering sweep."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
        self.SetSteering(0.15 * math.sin(0.4 * time))


driver = ScriptedDriver(veh_obj)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase cam + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM with onboard camera")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)


# === Main loop === render-cadence outer loop; Synchronize/Advance the full subsystem stack
frame = 0
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
            manager.Update()                       # pump the onboard camera every step

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)               # internally steps the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:          # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
