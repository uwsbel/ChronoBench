"""HMMWV on a multi-patch rigid terrain (Irrlicht).

Models a full HMMWV (High Mobility Multipurpose Wheeled Vehicle) driving across a
complex rigid terrain assembled from SEVERAL veh.RigidTerrain patches of different
surface types:
  * a large flat box patch (the driving lane, where the vehicle spawns),
  * a triangle-mesh patch built from a Wavefront bump mesh (terrain/meshes/bump.obj),
  * a heightmap patch built from a grayscale image (terrain/height_maps/bump64.bmp),
each carrying its own contact material and its own texture.

System type: NSC, owned by the veh.HMMWV_Full wrapper. Contact is resolved by the
Bullet collision system. The vehicle uses the TMEASY tire model, a SHAFTS engine,
an AUTOMATIC_SHAFTS transmission, AWD driveline, and a Pitman-arm steering. A
scripted veh.ChDriver subclass releases the brake, applies a modest throttle, and
adds a gentle steering oscillation so the HMMWV accelerates forward and remains
within the flat patch extents (it does not run off into the void).

Expected behavior: the vehicle settles on its tires, drives forward along +X on the
flat patch staying upright, while the mesh bump patch and the heightmap patch are
visible alongside as distinct textured surfaces.
"""

# === Imports ===
import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics) — no bare literals downstream ===
TIME_STEP = 2e-3                       # integration + tire step
SIM_END = 8.0                          # seconds of simulated time (keeps drive on-patch)
RENDER_FPS = 50.0                      # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# Flat driving patch (X length, Y width) centered at world origin.
FLAT_LEN = 80.0
FLAT_WID = 30.0
FLAT_Z = 0.0                           # top surface of the flat patch

# Mesh bump patch and heightmap patch sit to either side of the lane (off the path).
# The bump mesh spans 64 m in Y, so its center is pushed far in +Y to clear the lane.
MESH_PATCH_Y = 50.0                    # +Y, bump mesh footprint stays above the flat lane
HMAP_LEN = 40.0
HMAP_WID = 20.0
HMAP_Y = -40.0                         # -Y, clear of the flat lane
HMAP_HMIN = 0.0
HMAP_HMAX = 2.0

# Vehicle spawn: well inside the flat patch, facing +X, leaving room to drive.
VEH_INIT_X = -28.0
VEH_INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5            # HMMWV chassis-origin height above wheel-bottom
VEH_INIT_Z = FLAT_Z + SUSPENSION_REF_HEIGHT
TIRE_RADIUS = 0.46                     # approx HMMWV tire radius (for footprint check)
ZTOL = 0.10                            # allowed wheel-bottom clearance/overlap vs flat top

# Vehicle configuration.
init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
init_rot = chrono.QuatFromAngleZ(0.0)  # heading along +X

# Driver schedule constants.
BRAKE_RELEASE_T = 0.8                  # brake until this time, then drive
DRIVE_THROTTLE = 0.30                  # modest throttle keeps it inside the patch
STEER_AMP = 0.04                       # gentle steering oscillation amplitude
STEER_FREQ = 1.2                       # rad/s; full cycles keep net heading straight


# === Driver: scripted veh.ChDriver subclass (time-based control law) ===
class ScriptedDriver(veh.ChDriver):
    """Releases the brake after a short settle, then applies a modest throttle and a
    gentle sinusoidal steering so the vehicle drives forward and stays on the lane."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < BRAKE_RELEASE_T:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
            self.SetSteering(STEER_AMP * math.sin(STEER_FREQ * time))


# === Vehicle data path ===
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")


# === Vehicle: HMMWV_Full wrapper (creates + owns its ChSystemNSC) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)        # prompt: TMEASY tire model
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# Mesh visualization on every vehicle component.
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()             # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()       # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()           # cache: vehicle subsystem, reused for spindle queries
# wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
# links created inside the wrapper; terrain: RigidTerrain patches added below.

# Collision system: REQUIRED for the vehicle+terrain contact. Set Bullet on the
# wrapper-owned system AFTER Initialize (the framework collision-system selection).
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain: a single veh.RigidTerrain with several distinct patches ===
terrain = veh.RigidTerrain(system)

# Patch 1 — large flat box (the driving lane), tiled asphalt-like texture.
flat_mat = chrono.ChContactMaterialNSC()
flat_mat.SetFriction(0.9)
flat_mat.SetRestitution(0.01)
flat_patch = terrain.AddPatch(
    flat_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, FLAT_Z), chrono.QUNIT),
    FLAT_LEN, FLAT_WID,
)
flat_patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
flat_patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

# Patch 2 — triangle-mesh patch from a Wavefront bump mesh, grass texture.
mesh_mat = chrono.ChContactMaterialNSC()
mesh_mat.SetFriction(0.9)
mesh_mat.SetRestitution(0.01)
mesh_patch = terrain.AddPatch(
    mesh_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, MESH_PATCH_Y, FLAT_Z), chrono.QUNIT),
    veh.GetVehicleDataFile("terrain/meshes/bump.obj"),
)
mesh_patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/grass.jpg"), 6, 6)

# Patch 3 — heightmap patch from a grayscale image, dirt texture.
hmap_mat = chrono.ChContactMaterialNSC()
hmap_mat.SetFriction(0.9)
hmap_mat.SetRestitution(0.01)
hmap_patch = terrain.AddPatch(
    hmap_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, HMAP_Y, FLAT_Z), chrono.QUNIT),
    veh.GetVehicleDataFile("terrain/height_maps/bump64.bmp"),
    HMAP_LEN, HMAP_WID, HMAP_HMIN, HMAP_HMAX,
)
hmap_patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 16, 16)

terrain.Initialize()                   # initialize AFTER all patches are added

# === Footprint check: wheels rest on (not through) the flat patch ===
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= FLAT_Z - ZTOL, (
    f"vehicle sinks into flat patch: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs patch top z={FLAT_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{FLAT_Z - wheel_bottom_z:.3f} m"
)

# === Driver: scripted, autonomous (no human-in-the-loop) ===
driver = ScriptedDriver(veh_obj)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Multi-Patch Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)               # input-bar HUD reflects the scripted inputs


# === Main loop === scripted drive; vehicle subsystems advance the wrapper system
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)        # internally steps the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
