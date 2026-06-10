"""HMMWV climbing a rigid height-mapped hill.

Model: a full HMMWV wheeled vehicle (TMEASY tires, AWD, shafts powertrain)
driving up a single rigid terrain patch generated from a height map (a flat
low apron that ramps up to a raised plateau — the hill). The vehicle is given
full throttle and climbs from the low apron up the slope onto the crest.

System type: NSC (Non-Smooth Contacts) — the vehicle wrapper owns the
ChSystemNSC; the rigid terrain and its Bullet contacts are attached to it.
Main bodies: HMMWV chassis + four spindles/wheels (created by the wrapper),
and the rigid height-mapped terrain patch.
Expected behavior: the chassis translates forward and gains elevation as it
drives up the hill, ending on the crest while still on the patch.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / driving schedule (no bare literals downstream)
time_step = 2e-3                      # integration step (s)
tire_step = 1e-3                      # TMEASY tire substep (s)
sim_end = 6.5                         # ends on the raised crest, still on-patch (s)

TERRAIN_LENGTH = 64.0                 # patch X extent (m) — square height-map patch
TERRAIN_WIDTH = 64.0                  # patch Y extent (m)
HILL_MIN_Z = 0.0                      # height-map low (black) apron elevation (m)
HILL_MAX_Z = 4.0                      # height-map high (white) crest elevation (m)

SUSPENSION_REF_HEIGHT = 0.6           # HMMWV chassis-origin height above wheel-bottom at rest (m)
# The height map is a flat low apron (z = HILL_MIN_Z) over the negative-X half
# that ramps up to a raised plateau (z = HILL_MAX_Z) over the positive-X half.
# Spawn on the flat apron and climb the slope toward the crest. Height-mapped
# mesh patches report GetHeight()==0 everywhere, so the spawn elevation is taken
# from the known apron map height, not from terrain.GetHeight().
HILL_FOOT_X = -15.0                   # spawn on the flat low apron, facing the slope (m)
HILL_FOOT_Z = HILL_MIN_Z              # apron map surface elevation at the spawn (m)
SPAWN_Y = 0.0                         # drive straight up the centreline (m)
SPAWN_Z = HILL_FOOT_Z + SUSPENSION_REF_HEIGHT   # chassis-origin world Z on the apron (m)

render_fps = 50.0                                                # review cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))    # precomputed once: steps per frame

init_loc = chrono.ChVector3d(HILL_FOOT_X, SPAWN_Y, SPAWN_Z)
init_rot = chrono.QUNIT                # facing +X, toward the crest


# === Scripted driver === full-throttle climb (no human-in-the-loop, headless-safe)
class HillClimbDriver(veh.ChDriver):
    """Scripted driver: brief settle, then sustained full throttle, no steering."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)      # let the suspension settle on the patch
            self.SetBraking(1.0)
        else:
            self.SetThrottle(1.0)      # full throttle to climb the hill
            self.SetBraking(0.0)
        self.SetSteering(0.0)


# === Error-handled build + run === guard output dir + solver/run so partial output flushes
try:
    # === Vehicle (HMMWV_Full wrapper owns the ChSystemNSC) ===
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC contact method
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # TMEASY tires grip the rigid hill
    hmmwv.SetTireStepSize(tire_step)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    system = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
    # Bullet collision is REQUIRED for the vehicle/terrain contacts. Set it on the
    # wrapper-owned system AFTER Initialize (the wrapper collision system now exists).
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chassis = hmmwv.GetChassisBody()            # cache: main chassis body, reused every step
    veh_obj = hmmwv.GetVehicle()                # cache: vehicle subsystem, reused every step
    # spindles/wheels: veh_obj.GetAxle(i)... ; joints: suspension + steering links in wrapper

    # === Terrain === single rigid patch from a convex height map (the hill), built
    # AFTER vehicle.Initialize() so the wrapper collision system is already in place.
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()   # NSC material to match the NSC system
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,                        # patch centred at origin, no rotation
        veh.GetVehicleDataFile("terrain/height_maps/slope.bmp"),
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
        HILL_MIN_Z,
        HILL_MAX_Z,
    )
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 16, 16)
    patch.SetColor(chrono.ChColor(0.7, 0.6, 0.45))
    terrain.Initialize()

    # === Footprint check === wheels must start on (not through) the low edge of the hill
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    TIRE_RADIUS = 0.46                           # HMMWV TMEASY tire radius (m)
    ZTOL = 0.3                                    # allowed clearance/overlap at spawn (m)
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= HILL_FOOT_Z - ZTOL, (
        f"vehicle spawns through the hill: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs slope-foot surface z={HILL_FOOT_Z:.3f}; raise SUSPENSION_REF_HEIGHT"
    )

    # === Driver === scripted full-throttle climb
    driver = HillClimbDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full vehicle Irrlicht scene: window + chase cam + sky + lights + logo
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV Hill Climb")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.6)   # follow point, distance, height
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()                              # standard outdoor sky backdrop
    vis.AddTypicalLights()                       # standard lighting
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)                     # throttle/brake HUD bars


    # === Main loop === render once per frame; advance the full vehicle stack per step
    while vis.Run() and system.GetChTime() < sim_end:
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

            driver.Advance(time_step)
            terrain.Advance(time_step)
            hmmwv.Advance(time_step)            # advances the wrapper-owned system
            vis.Advance(time_step)
            if system.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === flush CSV, assemble review video + plot, drop leftover frames
