"""HMMWV driving on a rigid highway with an extra side terrain patch.

Models a full HMMWV wheeled vehicle (TMEASY tires) driving straight ahead on a
rigid-terrain highway. The scene uses three RigidTerrain patches:
  * a large flat drive lane (the straight, level surface the vehicle rolls on so
    it stays on-road and upright),
  * the SynChrono "Highway" mesh as a visual+collision road feature alongside,
  * an extra mesh patch built from `bump.obj`, located at (0, -42, 0), colored
    (0.5, 0.5, 0.8) and textured with dirt.jpg (UV scale 6.0 x 6.0) — a side
    feature off the drive lane.

System type: NSC (the HMMWV_Full wrapper owns a ChSystemNSC). Collision is
resolved by Bullet (set explicitly after vehicle initialization). A scripted
ChDriver subclass applies forward throttle with centered (zero) steering so the
vehicle tracks straight down the flat lane. Expected behavior: the vehicle
accelerates and translates forward along +X while remaining upright.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics) ===
TIME_STEP = 2.0e-3                  # integration step (s)
TIRE_STEP = 1.0e-3                  # tire substep (s)
SIM_END = 12.0                      # total simulated time (s)
RENDER_FPS = 50.0                   # review-video frame rate

DRIVE_LANE_LEN = 200.0              # flat drive-lane length along X (m)
DRIVE_LANE_WID = 40.0               # flat drive-lane width along Y (m)
TERRAIN_TOP_Z = 0.0                 # top surface of the flat drive lane (m)

SUSPENSION_REF_HEIGHT = 0.5         # HMMWV chassis origin above wheel-bottom at rest (m)
TIRE_RADIUS = 0.46                  # HMMWV tire radius for footprint assert (m)
ZTOL = 0.10                         # allowed wheel-bottom clearance/overlap vs lane top (m)

VEH_INIT_X = -80.0                  # spawn near the start of the lane (m)
VEH_INIT_Y = 0.0                    # centered on the drive lane (m)
VEH_INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived chassis-origin height

# Extra side patch (from the prompt): bump mesh placed off the drive lane.
BUMP_POS = chrono.ChVector3d(0.0, -42.0, 0.0)        # patch location (m)
BUMP_COLOR = chrono.ChColor(0.5, 0.5, 0.8)           # patch surface color
BUMP_TEX_SCALE = (6.0, 6.0)                          # dirt.jpg UV tiling

THROTTLE_RAMP_END = 1.0             # ramp throttle up over the first second (s)
CRUISE_THROTTLE = 0.6               # steady forward throttle

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


# === Driver (scripted, centered steering) ===
class StraightDriver(veh.ChDriver):
    """Scripted driver: ramp throttle to cruise, zero (centered) steering."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < THROTTLE_RAMP_END:
            self.SetThrottle(CRUISE_THROTTLE * (time / THROTTLE_RAMP_END))
        else:
            self.SetThrottle(CRUISE_THROTTLE)
        self.SetBraking(0.0)
        self.SetSteering(0.0)          # centered steering -> straight line


def main():
    # === Vehicle (HMMWV_Full, TMEASY tires) ===
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(
            chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT
        )
    )
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)     # prompt context: deformable highway tires
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    sys = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
    chassis = hmmwv.GetChassisBody()        # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()            # cache: vehicle subsystem handle, reused for spindles
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
    # links are created inside the wrapper; terrain patches are added below.

    # Bullet collision is required for the vehicle <-> terrain contact. Set it on
    # the wrapper-owned system right after Initialize (never create a new system).
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Terrain (rigid: flat drive lane + highway mesh + extra bump patch) ===
    terrain = veh.RigidTerrain(sys)

    lane_mat = chrono.ChContactMaterialNSC()
    lane_mat.SetFriction(0.9)
    lane_mat.SetRestitution(0.01)
    lane_patch = terrain.AddPatch(
        lane_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z), chrono.QUNIT),
        DRIVE_LANE_LEN,
        DRIVE_LANE_WID,
    )
    lane_patch.SetTexture(
        veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200
    )
    lane_patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

    # Highway mesh as a visual+collision road feature (collision + visual meshes).
    highway_mat = chrono.ChContactMaterialNSC()
    highway_mat.SetFriction(0.9)
    highway_mat.SetRestitution(0.01)
    highway_patch = terrain.AddPatch(
        highway_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z), chrono.QUNIT),
        chrono.GetChronoDataFile("synchrono/meshes/Highway_col.obj"),
        True,
        0.0,
        False,                       # collision mesh only; visual mesh added next
    )
    highway_vis = chrono.ChVisualShapeModelFile()
    highway_vis.SetFilename(chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj"))
    highway_patch.GetGroundBody().AddVisualShape(
        highway_vis, chrono.ChFramed(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z), chrono.QUNIT)
    )

    # Extra patch from the prompt: bump.obj mesh at (0, -42, 0), colored + textured.
    bump_mat = chrono.ChContactMaterialNSC()
    bump_mat.SetFriction(0.9)
    bump_mat.SetRestitution(0.01)
    bump_patch = terrain.AddPatch(
        bump_mat,
        chrono.ChCoordsysd(BUMP_POS, chrono.QUNIT),
        veh.GetVehicleDataFile("terrain/meshes/bump.obj"),
    )
    bump_patch.SetColor(BUMP_COLOR)
    bump_patch.SetTexture(
        veh.GetVehicleDataFile("terrain/textures/dirt.jpg"),
        BUMP_TEX_SCALE[0],
        BUMP_TEX_SCALE[1],
    )

    terrain.Initialize()

    # === Footprint assert (wheel bottoms rest on the flat lane after Initialize) ===
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into lane: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs lane top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Driver (scripted straight-line control) ===
    driver = StraightDriver(veh_obj)
    driver.Initialize()

    # === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV on Rigid Highway")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 10, -10, 4),
                  chrono.ChVector3d(VEH_INIT_X, 0, 0.5))
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)

    # === Main loop === throttled render + Synchronize/Advance subsystem stepping

    frame = 0
    try:
        while vis.Run() and sys.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(RENDER_EVERY):
                sim_time = sys.GetChTime()
                driver_inputs = driver.GetInputs()
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, driver_inputs, terrain)
                vis.Synchronize(sim_time, driver_inputs)
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)        # internally steps the wrapper-owned system
                vis.Advance(TIME_STEP)
                if sys.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        pass

    # === Post-processing === assemble review video + plot, then clean frames


if __name__ == "__main__":
    main()
