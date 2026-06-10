"""MAN_5t military truck driving over rigid rolling hills (height-map terrain).

Model
-----
- System: NSC, owned by the ``veh.MAN_5t`` wheeled-vehicle wrapper
  (chassis, four axles with spindles, suspension + steering links, TMEASY tires).
- Terrain: a single ``veh.RigidTerrain`` patch whose surface is a height map of
  rolling hills (Bullet collision), textured with grass, built AFTER the vehicle
  is initialized so it can be attached to the wrapper-owned system.
- Driver: a scripted ``veh.ChDriver`` subclass — brief brake, then steady
  forward throttle so the truck climbs the hills.

Expected behavior
------------------
The truck spawns above the rolling-hill surface at world X=-20, Y=0, drops its
wheels onto the hills, then accelerates forward (+X), translating many metres and
riding up and over the rolling-hill profile. Chassis X-position increases over time.
"""

# === Imports ===
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics (no bare position literals downstream)
TIME_STEP = 2.0e-3                         # solver step (s)
SIM_END = 12.0                             # simulated duration (s)
RENDER_FPS = 50.0                          # review-video frame rate

# Rolling-hills rigid terrain (height-map patch).
TERRAIN_LENGTH = 100.0                     # patch X extent (m)
TERRAIN_WIDTH = 100.0                      # patch Y extent (m)
HILL_H_MIN = 0.0                           # height-map min elevation (m)
HILL_H_MAX = 3.0                           # height-map max elevation (m)

# Vehicle spawn (world frame) and orientation. X/Y come from the request; the
# rolling-hill crest near (-20, 0) sits ~2.7 m up, so spawn Z clears that crest
# (HILL_H_MAX + suspension reference) and lets the wheels drop onto the surface.
INIT_X = -20.0                             # spawn X (m)
INIT_Y = 0.0                               # spawn Y (m)
INIT_Z = HILL_H_MAX + 0.5                  # spawn Z (m): above the highest hill crest
INIT_LOC = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.QUNIT                    # facing +X
HEIGHTMAP_FILE = "vehicle/terrain/height_maps/terrain3.bmp"
GRASS_TEXTURE = "vehicle/terrain/textures/grass.jpg"

# Driver schedule.
BRAKE_UNTIL = 0.5                          # hold brake briefly so wheels settle
DRIVE_THROTTLE = 0.4                       # moderate throttle: climb hills, stay on patch

# Derived render cadence (precomputed once).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


# === Driver === scripted time-based control (no human-in-the-loop)
class HillClimbDriver(veh.ChDriver):
    """Brake briefly to settle, then apply steady forward throttle."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < BRAKE_UNTIL:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
        self.SetSteering(0.0)              # drive straight up the hills


def main():

    # === Vehicle === MAN_5t wrapper owns the ChSystem (NSC) + all its bodies
    truck = veh.MAN_5t()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisCollisionType(veh.CollisionType_NONE)
    truck.SetChassisFixed(False)
    truck.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    truck.SetEngineType(veh.EngineModelType_SIMPLE_MAP)               # MAN_5t powertrain map
    truck.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    truck.SetTireType(veh.TireModelType_TMEASY)        # handling tire that grips the hills
    truck.SetTireStepSize(TIME_STEP)
    truck.Initialize()

    truck.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    truck.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    truck.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.MAN_5t wrapper) ===
    system = truck.GetSystem()                 # cache: ChSystemNSC owned by wrapper, reused
    chassis = truck.GetChassisBody()           # cache: main chassis rigid body, reused each step
    veh_obj = truck.GetVehicle()               # cache: vehicle subsystem handle, reused
    # spindles/suspension/steering links: created inside the wrapper per axle.

    # Required for any contact/collision scene: set the Bullet collision system on
    # the wrapper-owned system so the rigid-terrain height-map contacts resolve.
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Terrain === rigid rolling hills from a height map, grass-textured
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,                       # patch centered at origin, Z-up
        chrono.GetChronoDataFile(HEIGHTMAP_FILE),
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
        HILL_H_MIN,
        HILL_H_MAX,
    )
    patch.SetTexture(chrono.GetChronoDataFile(GRASS_TEXTURE), 40, 40)
    terrain.Initialize()

    # Spawn-height sanity: the wheel bottoms must START above the highest possible
    # hill crest so the truck drops onto the surface instead of spawning buried in
    # a hill (which wedges it in contact and it never moves). Read actual spindle
    # world Z after Initialize and check it clears HILL_H_MAX.
    spindle_z = [veh_obj.GetSpindlePos(a, s).z
                 for a in range(veh_obj.GetNumberAxles())
                 for s in (veh.LEFT, veh.RIGHT)]
    assert min(spindle_z) > HILL_H_MAX, (
        f"spindles start at/below the hill crest: min spindle z={min(spindle_z):.3f} "
        f"vs max hill height z={HILL_H_MAX:.3f}; raise INIT_Z so the wheels clear the hills"
    )

    # === Driver === scripted climb (settle, then steady throttle)
    driver = HillClimbDriver(veh_obj)
    driver.Initialize()

    # === Visualization === vehicle-aware Irrlicht: window + sky + camera + lights + grid
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("MAN 5t on rolling hills")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.0), 12.0, 1.0)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddGrid(2.0, 2.0, 50, 50,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                chrono.ChColor(0.35, 0.35, 0.35))      # ground reference grid
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)

    # === Logging setup ===

    # === Main loop === render once per frame; advance physics in an inner batch
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
                truck.Synchronize(sim_time, driver_inputs, terrain)
                vis.Synchronize(sim_time, driver_inputs)
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                truck.Advance(TIME_STEP)       # advances the wrapper-owned system
                vis.Advance(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise

    # === Post-processing === flush CSV, assemble review video + plot, drop frame PNGs


if __name__ == "__main__":
    main()
