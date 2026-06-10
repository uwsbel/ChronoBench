"""HMMWV on a multi-patch rigid terrain.

Models a full HMMWV (TMEASY tires) driving gently across a flat rigid-terrain
patch, with three additional rigid-terrain patches present in the world: a
raised flat box patch, a Wavefront-mesh patch (test.obj), and a heightmap-bump
patch (bump64.bmp). System type is NSC (the HMMWV_Full wrapper owns it). The
vehicle spawns on the first (ground-level flat) patch and accelerates forward
under a gentle scripted driver, staying supported on its launch patch for the
whole run.

Expected behavior: the HMMWV settles onto the flat launch patch, then rolls
forward in a straight line at modest throttle; the four terrain patches are
visible at their distinct world positions with their own materials/textures.
"""

import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / driver parameters (no bare literals downstream)
TIME_STEP = 2e-3                      # integration step (s)
TIRE_STEP = 1e-3                      # TMEASY tire substep (s)
SIM_END = 5.0                         # total simulated time (s) — keeps the HMMWV on its launch patch
RENDER_FPS = 50.0                     # review-video frame rate

# Terrain patch sizes (m).
FLAT_LEN, FLAT_WID = 24.0, 16.0       # flat box patches (X by Y extent)
MESH_SWEEP = 0.01                     # collision sweep-sphere radius for mesh patch
HMAP_LEN, HMAP_WID = 24.0, 24.0       # heightmap patch footprint
HMAP_MIN, HMAP_MAX = 0.0, 1.0         # heightmap height range (m)

# Patch world positions (FINAL desired placements).
P1_POS = chrono.ChVector3d(-20, 5, 0)       # flat box, ground level — vehicle launch patch
P2_POS = chrono.ChVector3d(20, -5, 0.2)     # flat box, raised
P3_POS = chrono.ChVector3d(5, -45, 0)        # mesh patch (test.obj)
P4_POS = chrono.ChVector3d(10, 40, 0)        # heightmap bump patch (bump64.bmp)

# Vehicle spawn derived from the launch patch (patch 1 top is at z = P1_POS.z).
SUSPENSION_REF_HEIGHT = 0.5           # HMMWV chassis-origin height above wheel-bottom at rest
TIRE_RADIUS = 0.46                    # HMMWV tire radius (m), used for the footprint assert
ZTOL = 0.10                           # allowed wheel-bottom clearance/overlap vs patch top
PATCH1_TOP_Z = P1_POS.z               # flat patch surface height
VEH_INIT_X, VEH_INIT_Y = P1_POS.x, P1_POS.y
VEH_INIT_Z = PATCH1_TOP_Z + SUSPENSION_REF_HEIGHT
init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
init_rot = chrono.QUNIT               # face +X, drive forward along the launch patch


# === Scripted driver === gentle straight-line acceleration, no human input (headless)
class GentleDriver(veh.ChDriver):
    """Time-based open-loop driver: brief settle, then ease throttle to a modest cruise."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)          # let the suspension settle onto the patch
            self.SetBraking(1.0)
        else:
            ramp = min(1.0, (time - 0.5) / 2.0)
            self.SetThrottle(0.25 * ramp)  # modest throttle so the vehicle stays supported
            self.SetBraking(0.0)
        self.SetSteering(0.0)              # straight ahead


# === Vehicle === HMMWV_Full wrapper (NSC system created + owned internally)
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # prompt: drivable tire on rigid terrain
hmmwv.SetTireStepSize(TIRE_STEP)
hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()               # cache: vehicle subsystem handle, reused every step
# wheels/spindles: veh_obj.GetSpindlePos(axle, side); suspension + steering joints live in the wrapper

# Collision system MUST be Bullet for the rigid-terrain contacts (vehicle scene has contact).
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === four rigid patches, each with its own material + texture
terrain = veh.RigidTerrain(system)

# Patch 1 — flat box at ground level (vehicle launch patch).
mat1 = chrono.ChContactMaterialNSC()
mat1.SetFriction(0.9)
mat1.SetRestitution(0.01)
patch1 = terrain.AddPatch(mat1, chrono.ChCoordsysd(P1_POS, chrono.QUNIT), FLAT_LEN, FLAT_WID)
patch1.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 16, 16)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

# Patch 2 — flat box, raised.
mat2 = chrono.ChContactMaterialNSC()
mat2.SetFriction(0.9)
mat2.SetRestitution(0.01)
patch2 = terrain.AddPatch(mat2, chrono.ChCoordsysd(P2_POS, chrono.QUNIT), FLAT_LEN, FLAT_WID)
patch2.SetTexture(veh.GetVehicleDataFile("terrain/textures/concrete.jpg"), 16, 16)
patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.75))

# Patch 3 — Wavefront-mesh patch (test.obj).
mat3 = chrono.ChContactMaterialNSC()
mat3.SetFriction(0.9)
mat3.SetRestitution(0.01)
patch3 = terrain.AddPatch(
    mat3, chrono.ChCoordsysd(P3_POS, chrono.QUNIT),
    veh.GetVehicleDataFile("terrain/meshes/test.obj"), True, MESH_SWEEP,
)
patch3.SetTexture(veh.GetVehicleDataFile("terrain/textures/grass.jpg"), 8, 8)

# Patch 4 — heightmap bump patch (bump64.bmp).
mat4 = chrono.ChContactMaterialNSC()
mat4.SetFriction(0.9)
mat4.SetRestitution(0.01)
patch4 = terrain.AddPatch(
    mat4, chrono.ChCoordsysd(P4_POS, chrono.QUNIT),
    veh.GetVehicleDataFile("terrain/height_maps/bump64.bmp"),
    HMAP_LEN, HMAP_WID, HMAP_MIN, HMAP_MAX,
)
patch4.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 16, 16)

terrain.Initialize()   # finalize all four patches together

# === Footprint assert === confirm the wheels start on (not through) the launch patch
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= PATCH1_TOP_Z - ZTOL, (
    f"vehicle sinks into launch patch: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs patch top z={PATCH1_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{PATCH1_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === gentle scripted forward-acceleration controller
driver = GentleDriver(veh_obj)
driver.Initialize()

# === Visualization === full vehicle-aware Irrlicht scene: window + chase cam + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on multi-patch rigid terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.6)   # chase-cam track point/distance/height
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 10, VEH_INIT_Y - 10, 6),
              chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, 0))   # initial overview eye/target
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Derived loop constants === computed once before the loop
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


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
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)        # internally steps the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
