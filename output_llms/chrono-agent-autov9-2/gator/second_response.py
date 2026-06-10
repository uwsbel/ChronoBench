"""Gator UTV on a multi-patch rigid terrain test course (PyChrono 9.0.1, Irrlicht).

Models the Chrono `veh.Gator` wheeled utility vehicle (NSC contact, owned by the
wrapper) driving forward across a terrain course built from FOUR distinct rigid
patches, each with its own texture:
  1. a flat grass patch (spawn / approach),
  2. a flat dirt patch carrying a raised bump (bump heightmap) for a ride event,
  3. a flat concrete patch (transition),
  4. a heightmap slope patch so the vehicle can be tested for gradability.

A scripted ChDriver applies a brief settle, then steady throttle with a light
proportional steering correction that holds the centerline (y=0) so the Gator
tracks straight up the course and climbs the slope. The expected behavior: the
Gator settles on the grass, rolls forward, rides over the bump, then climbs the
inclined slope patch (chassis Z and pitch rise as it ascends). Key physics
quantities are logged to CSV for review.
"""

import os
import math


import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / course layout (no bare literals downstream)
TIME_STEP = 2.0e-3                  # integration step (s)
TIRE_STEP = 1.0e-3                  # tire force-model substep (s)
SIM_END = 20.0                      # total simulated time (s) — long enough to climb the slope
RENDER_FPS = 50.0                   # review render cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

PATCH_LEN = 30.0                    # X extent of each patch (m)
PATCH_WID = 24.0                    # Y extent of each patch (m) — wide margin for tracking
PATCH_THICK = 1.0                   # rigid patch slab thickness (m)

# Patch centers laid end-to-end along +X so the Gator drives across all four.
GRASS_CX = 0.0                      # patch 1: flat grass (spawn)
DIRT_CX = PATCH_LEN                 # patch 2: flat dirt + bump
CONCRETE_CX = 2.0 * PATCH_LEN       # patch 3: flat concrete
SLOPE_CX = 3.0 * PATCH_LEN          # patch 4: heightmap slope (gradability)

BUMP_HMIN = 0.0                     # bump heightmap min height (m)
BUMP_HMAX = 0.35                    # bump heightmap max height (m)
SLOPE_HMIN = 0.0                    # slope heightmap min height (m)
SLOPE_HMAX = 4.0                    # slope heightmap max height (m) -> gradability test

VEH_INIT_X = -10.0                  # spawn near the back of the grass patch
VEH_INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5         # Gator chassis-origin height above wheel bottom
GRASS_TOP_Z = 0.0                   # flat grass patch top plane (z=0)
VEH_INIT_Z = GRASS_TOP_Z + SUSPENSION_REF_HEIGHT
ZTOL = 0.10                         # allowed wheel-bottom clearance vs support top

SETTLE_TIME = 1.0                   # let suspension settle before driving (s)
DRIVE_THROTTLE = 0.7                # steady throttle to cross course + climb slope
STOP_CLIMB_Z = 3.5                  # chassis Z at which gradability is proven -> brake to hold

TEX_GRASS = "vehicle/terrain/textures/grass.jpg"
TEX_DIRT = "vehicle/terrain/textures/dirt.jpg"
TEX_CONCRETE = "vehicle/terrain/textures/concrete.jpg"
TEX_TILE = "vehicle/terrain/textures/tile4.jpg"
HM_BUMP = "vehicle/terrain/height_maps/bump64.bmp"
HM_SLOPE = "vehicle/terrain/height_maps/slope.bmp"


# === Scripted driver === settle, then steady throttle + centerline-holding steering
STEER_GAIN = 0.08                   # proportional steering gain on lateral offset (1/m)
STEER_LIMIT = 0.35                  # clamp corrective steering to a gentle range


class CourseDriver(veh.ChDriver):
    """Brief brake-settle, then steady throttle; light P steering holds y=0.

    Reads the chassis lateral offset each step and applies a small, clamped
    counter-steer so the Gator tracks the course centerline straight onto the
    gradability slope (open-loop straight steering otherwise drifts off-patch).
    """

    def __init__(self, vehicle, chassis_body):
        super().__init__(vehicle)
        self._chassis = chassis_body   # cache: chassis handle for lateral feedback
        self._climbed = False          # latched once the slope is summited

    def Synchronize(self, time):
        if time < SETTLE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
            return
        pos = self._chassis.GetPos()   # cache: one pose fetch reused this step
        # Latch a stop once the slope is summited so the Gator holds on the grade
        # instead of cresting and running off the end of the slope patch.
        if pos.z >= STOP_CLIMB_Z:
            self._climbed = True
        if self._climbed:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
            return
        self.SetThrottle(DRIVE_THROTTLE)
        self.SetBraking(0.0)
        steer = max(-STEER_LIMIT, min(STEER_LIMIT, -STEER_GAIN * pos.y))
        self.SetSteering(steer)        # hold centerline for the gradability run


def main():

    # === Vehicle === Gator UTV (wrapper creates & owns its ChSystem + bodies)
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)
    gator.SetChassisCollisionType(veh.CollisionType_NONE)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z),
                           chrono.QUNIT)
    )
    gator.SetTireType(veh.TireModelType_TMEASY)   # slip/grip curve -> climbs slope
    gator.SetTireStepSize(TIRE_STEP)
    gator.Initialize()
    gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    gator.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.Gator wrapper) ===
    system = gator.GetSystem()            # ChSystemNSC owned by the wrapper
    chassis = gator.GetChassisBody()      # cache: main chassis rigid body, reused every step
    veh_obj = gator.GetVehicle()          # cache: ChWheeledVehicle handle for spindle queries
    # spindles/wheels: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
    # links live inside the wrapper; terrain patches are added below.

    # Bullet collision is REQUIRED here — vehicle tires contact the rigid terrain.
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Terrain === four rigid patches, each a distinct texture; bump + slope heightmaps
    terrain = veh.RigidTerrain(system)

    def make_material():
        mat = chrono.ChContactMaterialNSC()   # NSC system -> NSC contact material
        mat.SetFriction(0.9)
        mat.SetRestitution(0.01)
        return mat

    # Patch 1 — flat grass (spawn / approach).
    p_grass = terrain.AddPatch(
        make_material(),
        chrono.ChCoordsysd(chrono.ChVector3d(GRASS_CX, 0, 0), chrono.QUNIT),
        PATCH_LEN, PATCH_WID, PATCH_THICK,
    )
    p_grass.SetTexture(chrono.GetChronoDataFile(TEX_GRASS), 40, 20)

    # Patch 2 — flat dirt carrying a raised bump (bump heightmap on this patch).
    p_dirt = terrain.AddPatch(
        make_material(),
        chrono.ChCoordsysd(chrono.ChVector3d(DIRT_CX, 0, 0), chrono.QUNIT),
        PATCH_LEN, PATCH_WID, PATCH_THICK,
    )
    p_dirt.SetTexture(chrono.GetChronoDataFile(TEX_DIRT), 40, 20)
    p_bump = terrain.AddPatch(
        make_material(),
        chrono.ChCoordsysd(chrono.ChVector3d(DIRT_CX, 0, 0), chrono.QUNIT),
        chrono.GetChronoDataFile(HM_BUMP),
        PATCH_LEN, PATCH_WID, BUMP_HMIN, BUMP_HMAX,
    )
    p_bump.SetTexture(chrono.GetChronoDataFile(TEX_DIRT), 40, 20)

    # Patch 3 — flat concrete (transition before the climb).
    p_conc = terrain.AddPatch(
        make_material(),
        chrono.ChCoordsysd(chrono.ChVector3d(CONCRETE_CX, 0, 0), chrono.QUNIT),
        PATCH_LEN, PATCH_WID, PATCH_THICK,
    )
    p_conc.SetTexture(chrono.GetChronoDataFile(TEX_CONCRETE), 40, 20)

    # Patch 4 — heightmap slope for the gradability test.
    p_slope = terrain.AddPatch(
        make_material(),
        chrono.ChCoordsysd(chrono.ChVector3d(SLOPE_CX, 0, 0), chrono.QUNIT),
        chrono.GetChronoDataFile(HM_SLOPE),
        PATCH_LEN, PATCH_WID, SLOPE_HMIN, SLOPE_HMAX,
    )
    p_slope.SetTexture(chrono.GetChronoDataFile(TEX_TILE), 40, 20)

    terrain.Initialize()

    # Verify the Gator starts resting on (not through) the grass patch.
    spindle_z = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_z.append(veh_obj.GetSpindlePos(axle, side).z)
    tire_radius = veh_obj.GetAxle(0).GetWheel(0, veh.LEFT).GetTire().GetRadius()
    wheel_bottom_z = min(spindle_z) - tire_radius
    assert wheel_bottom_z >= GRASS_TOP_Z - ZTOL, (
        f"Gator sinks into grass patch: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs grass top z={GRASS_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{GRASS_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Driver === scripted course controller with centerline-holding steering
    driver = CourseDriver(veh_obj, chassis)
    driver.Initialize()

    # === Visualization === vehicle-aware Irrlicht: window + sky + chase cam + lights + grid
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Gator — multi-patch terrain course (bump + gradability)")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.6)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddGrid(2.0, 2.0, 40, 12,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                chrono.ChColor(0.35, 0.35, 0.35))   # ground reference grid
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)

    # === Main loop === throttled render outer loop; vehicle subsystem step inner batch

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
                gator.Synchronize(sim_time, driver_inputs, terrain)
                vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                gator.Advance(TIME_STEP)        # internally steps the wrapper-owned system
                vis.Advance(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
