"""HMMWV full vehicle following a circular path on flat rigid terrain.

Model
-----
- Wrapper-managed `veh.HMMWV_Full` (SMC contact, AWD driveline, Pitman-arm
  steering, TMEASY tires) spawned on a 200 x 200 m flat `veh.RigidTerrain`
  patch. The terrain length is enlarged to 200 m so the full circular path
  fits inside the patch footprint.
- An autonomous path-following driver steers the vehicle around a circular
  Bezier path (`veh.CirclePath`) using a PID steering controller, while the
  throttle is held at a constant 0.3 (no speed controller authority over the
  throttle channel). Steering is the only closed-loop channel.
- The controller's sentinel (look-ahead) point and target (path) point are
  drawn as two colored spheres so the path being tracked is visible.

System type: NSC-vehicle wrapper owns a ChSystemSMC internally (SMC contact).
Expected behavior: the HMMWV accelerates under constant throttle and the PID
steering controller turns the wheels to keep the vehicle on the circular path,
tracing a closed loop within the terrain patch.
"""

import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Constants === geometry / physics / control parameters (no bare literals downstream)
TIME_STEP = 2e-3                       # integration step (s)
TIRE_STEP = 1e-3                       # tire substep (s)
SIM_END = 30.0                         # simulation duration (s)
RENDER_FPS = 50.0                      # review-video frame rate

TERRAIN_LENGTH = 200.0                 # X extent of the rigid patch (m) — enlarged so the path fits
TERRAIN_WIDTH = 200.0                  # Y extent of the rigid patch (m)
TERRAIN_HEIGHT = 0.0                   # top surface Z of the flat patch (m)

PATH_RADIUS = 30.0                     # circular path radius (m) — fits inside the 200 m terrain
PATH_RUN_IN = 20.0                     # straight run-in before the arc begins (m)
PATH_LAPS = 5                          # number of full circles to trace

CONST_THROTTLE = 0.3                   # constant throttle command (prompt)
STEER_KP = 0.8                         # PID steering proportional gain
STEER_KI = 0.0                         # PID steering integral gain
STEER_KD = 0.05                        # PID steering derivative gain
LOOK_AHEAD = 5.0                       # steering look-ahead distance (m)

SUSPENSION_REF_HEIGHT = 0.5            # chassis origin above wheel-bottom at rest (HMMWV ~0.5 m)
INIT_X = 0.0                           # spawn X (path starts here)
INIT_Y = 0.0                           # spawn Y
INIT_Z = TERRAIN_HEIGHT + SUSPENSION_REF_HEIGHT   # derived chassis-origin height
MARKER_RADIUS = 0.25                   # sentinel/target sphere radius (m)

# precomputed once: render cadence (physics steps between rendered frames)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))


class ConstantThrottlePathDriver(veh.ChPathFollowerDriver):
    """Path follower whose steering is PID-controlled but whose throttle is held
    constant. Overriding Synchronize lets the base class compute the steering
    command from the path, then we pin throttle/brake to the requested values."""

    def __init__(self, vehicle, path, name, throttle):
        # the base path-follower needs a target speed; it is irrelevant here
        # because we overwrite the throttle channel every step.
        super().__init__(vehicle, path, name, 1.0)
        self._throttle = throttle      # cache: constant throttle command

    def Synchronize(self, time):
        super().Synchronize(time)      # PID steering controller updates steering
        self.SetThrottle(self._throttle)
        self.SetBraking(0.0)


def build_marker(system, color):
    """Create a fixed body carrying a colored sphere visual, used to mark a
    steering-controller point. Returns the body so the loop can reposition it."""
    marker = chrono.ChBody()
    marker.SetFixed(True)
    sphere = chrono.ChVisualShapeSphere(MARKER_RADIUS)
    sphere.SetColor(color)
    marker.AddVisualShape(sphere, chrono.ChFramed())
    system.AddBody(marker)
    return marker


def main():

    # === Vehicle (wrapper creates and owns its ChSystemSMC) ===
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # grippy slip model for rigid road
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()
    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    system = hmmwv.GetSystem()                    # ChSystemSMC owned by the wrapper
    chassis = hmmwv.GetChassisBody()              # cache: main chassis rigid body, reused each step
    veh_obj = hmmwv.GetVehicle()                  # cache: ChWheeledVehicle, reused for spindle reads
    # wheels/spindles: veh_obj.GetAxle(i)...; joints (suspension + steering) live inside the wrapper

    # Collision system: REQUIRED for the vehicle/terrain contact pair.
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Terrain === flat rigid patch, 200 x 200 m, so the circular path fits
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Verify the wheels rest on (not through) the rigid patch after Initialize.
    spindle_z = [veh_obj.GetSpindlePos(a, s).z
                 for a in range(veh_obj.GetNumberAxles())
                 for s in (veh.LEFT, veh.RIGHT)]
    tire_radius = veh_obj.GetAxles()[0].GetWheels()[0].GetTire().GetRadius()
    wheel_bottom_z = min(spindle_z) - tire_radius
    assert wheel_bottom_z >= TERRAIN_HEIGHT - 0.1, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_HEIGHT:.3f}; raise SUSPENSION_REF_HEIGHT"
    )

    # === Path & driver === circular Bezier path + PID-steering, constant-throttle driver
    path_start = chrono.ChVector3d(INIT_X, INIT_Y, TERRAIN_HEIGHT + 0.5)
    path = veh.CirclePath(path_start, PATH_RADIUS, PATH_RUN_IN, True, PATH_LAPS)
    driver = ConstantThrottlePathDriver(veh_obj, path, "circle_path", CONST_THROTTLE)
    steer_ctrl = driver.GetSteeringController()   # cache: fetched once, reused every frame
    steer_ctrl.SetLookAheadDistance(LOOK_AHEAD)
    steer_ctrl.SetGains(STEER_KP, STEER_KI, STEER_KD)
    driver.Initialize()

    # Two balls marking the controller points: sentinel (look-ahead) + target (path).
    sentinel_marker = build_marker(system, chrono.ChColor(0.0, 1.0, 0.0))   # green sentinel
    target_marker = build_marker(system, chrono.ChColor(1.0, 0.0, 0.0))     # red target

    # === Visualization === vehicle-aware Irrlicht window: window + sky + camera + lights + grid
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV circular path follower")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 1.0)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddGrid(2.0, 2.0, 60, 60,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))    # ground reference grid
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)                       # steering/throttle/brake HUD bars


    # === Main loop === render-cadence outer loop; Synchronize/Advance the subsystem stack
    frame = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            # keep the two marker balls on the live controller points
            sentinel_marker.SetPos(steer_ctrl.GetSentinelLocation())
            target_marker.SetPos(steer_ctrl.GetTargetLocation())
            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                driver.Synchronize(sim_time)       # PID steering + constant throttle computed here
                driver_inputs = driver.GetInputs()  # read AFTER Synchronize so commands are current
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, driver_inputs, terrain)
                vis.Synchronize(sim_time, driver_inputs)
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)          # advances the wrapper-owned system
                vis.Advance(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:      # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        pass

    # === Post-processing === build review video + plot, then drop raw frames


if __name__ == "__main__":
    main()
