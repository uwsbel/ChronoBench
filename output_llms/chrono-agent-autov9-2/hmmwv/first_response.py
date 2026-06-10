"""Full HMMWV on flat rigid terrain (PyChrono 9.0.x, Irrlicht).

Models a complete HMMWV (High Mobility Multipurpose Wheeled Vehicle) driving on
a flat RigidTerrain patch. The vehicle wrapper owns an SMC ChSystem; the chassis,
suspension, steering, wheels, and TMEASY tires are created internally and rendered
with primitive (PRIMITIVES) visualization. A scripted driver applies throttle,
steering, and braking so the chassis accelerates forward and steers — the run is
batch/headless, so a programmatic driver is used in place of keyboard input.

System type: SMC (contact-method requested for the vehicle).
Main bodies: HMMWV chassis + 4 wheel spindles (wrapper-created), rigid terrain patch.
Expected behavior: the vehicle accelerates forward from rest, stays upright, and
follows a gentle steering input on the flat terrain.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Named constants === geometry / physics / driving schedule
TIME_STEP = 2e-3                       # integration + tire step (s)
TIRE_STEP = 1e-3                       # TMEASY tire substep (s)
SIM_END = 12.0                         # total simulated time (s)
RENDER_FPS = 50.0                      # real-time review cadence (frames / s)

TERRAIN_LENGTH = 200.0                 # rigid terrain X extent (m)
TERRAIN_WIDTH = 200.0                  # rigid terrain Y extent (m)
TERRAIN_HEIGHT = 0.0                   # flat terrain top plane (m)

INIT_X = 0.0                           # vehicle spawn X (m)
INIT_Y = 0.0                           # vehicle spawn Y (m)
SUSPENSION_REF_HEIGHT = 0.5            # chassis origin above wheel-bottom at rest (m)
INIT_Z = TERRAIN_HEIGHT + SUSPENSION_REF_HEIGHT   # derived chassis spawn Z
TIRE_RADIUS = 0.464                    # HMMWV TMEASY tire radius (m), for footprint check
ZTOL = 0.10                            # allowed wheel-bottom clearance vs terrain top (m)

# Derived render cadence — precomputed once (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


# === Driver === scripted throttle/steering/braking (interactive input is zero headless)
class ScriptedDriver(veh.ChDriver):
    """Time-based driver: brief settle, then accelerate with a gentle steer."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Hold still briefly so the suspension settles, then accelerate forward.
        if time < 1.0:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
        # Gentle sinusoidal steering so the chassis visibly turns.
        self.SetSteering(0.25 * math.sin(0.4 * time))


def main():
    # === Vehicle === full HMMWV wrapper: chassis + suspension + steering + TMEASY tires
    init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)        # identity: facing +X

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)            # prompt: contact method
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))  # prompt: location + orientation
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                   # prompt: TMEASY tire model
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    # Primitive visualization for every vehicle component (prompt: primitive viz).
    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    sys = hmmwv.GetSystem()                       # ChSystemSMC owned by the wrapper
    chassis = hmmwv.GetChassisBody()              # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()                  # cache: vehicle subsystem handle, reused every step
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
    # links are created inside the wrapper. Terrain patch body is added below.

    # Collision system: the scene has vehicle/terrain contact -> Bullet collision.
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Terrain === flat rigid patch with friction material and a tiled texture
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_HEIGHT), chrono.QUNIT),
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Footprint check === assert all wheel bottoms rest on the terrain after Initialize
    spindle_z = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_z.append(veh_obj.GetSpindlePos(axle, side).z)
    wheel_bottom_z = min(spindle_z) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_HEIGHT - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_HEIGHT:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_HEIGHT - wheel_bottom_z:.3f} m"
    )

    # === Driver === scripted throttle / steering / braking
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Full HMMWV on Rigid Terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)


    # === Main loop === render once per frame; advance physics in an inner batch
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


if __name__ == "__main__":
    main()
