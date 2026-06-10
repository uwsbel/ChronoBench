"""HMMWV vehicle simulation with a custom delayed scripted driver.

This NSC vehicle scene uses the PyChrono HMMWV full wrapper on flat rigid
terrain. A custom ChDriver subclass delays inputs by 0.5 s, ramps throttle to
0.7 after the delay, applies sinusoidal steering after 2 s, and stops the run at
4 s so the maneuver is visible and bounded.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Named values keep the vehicle setup and run objective explicit.
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 4.0
DRIVER_DELAY = 0.5
THROTTLE_RAMP_TIME = 0.2
TARGET_THROTTLE = 0.7
STEERING_START = 2.0
STEERING_AMPLITUDE = 0.35
STEERING_FREQUENCY = 0.75
TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 120.0
SUPPORT_TOP_Z = 0.0
SUSPENSION_REF_HEIGHT = 0.5
TIRE_RADIUS = 0.469
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once


# === Custom driver ===
# The prompt requires MyDriver to inherit ChDriver and control inputs by time.
class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay):
        super().__init__(vehicle)
        self.delay = delay

    def Synchronize(self, time):
        active_time = max(0.0, time - self.delay)
        if active_time <= 0.0:
            self.SetThrottle(0.0)
            self.SetBraking(0.15)
        else:
            throttle = min(TARGET_THROTTLE, TARGET_THROTTLE * active_time / THROTTLE_RAMP_TIME)
            self.SetThrottle(throttle)
            self.SetBraking(0.0)

        if time >= STEERING_START:
            steering = STEERING_AMPLITUDE * math.sin(2.0 * math.pi * STEERING_FREQUENCY * (time - STEERING_START))
        else:
            steering = 0.0
        self.SetSteering(steering)


def main():
    # === Vehicle and system ===
    # Catalog vehicle wrappers own the ChSystem; terrain and visualization share it.
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    init_pos = chrono.ChVector3d(0.0, 0.0, SUPPORT_TOP_Z + SUSPENSION_REF_HEIGHT)
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_pos, chrono.QUNIT))
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystem reused by terrain/logging
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chassis = hmmwv.GetChassisBody()  # cache: chassis body reused for placement/logging
    veh_obj = hmmwv.GetVehicle()  # cache: underlying wheeled vehicle queried repeatedly
    print("VEHICLE MASS: ", veh_obj.GetMass())

    # Wrapper-created components are explicit: system, chassis, wheels/spindles,
    # suspension/steering joints, powertrain, tires, and the visualization below.
    spindle_positions = []
    for axle_index in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_positions.append(veh_obj.GetSpindlePos(axle_index, side))
    wheel_bottom_z = min(p.z for p in spindle_positions) - TIRE_RADIUS
    assert wheel_bottom_z >= SUPPORT_TOP_Z - 0.08, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain z={SUPPORT_TOP_Z:.3f}"
    )

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain ===
    # RigidTerrain gives the HMMWV a flat contact surface for the timed maneuver.
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Visualization ===
    # Vehicle-specific Irrlicht visualizer follows the required initialize order.
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV custom delayed driver")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(veh_obj)

    # === Driver ===
    # MyDriver replaces the standard driver and owns the throttle/steering schedule.
    driver = MyDriver(veh_obj, DRIVER_DELAY)
    driver.Initialize()

    frame = 0
    realtime_timer = chrono.ChRealtimeStepTimer()

    # === Main loop ===
    # The loop synchronizes the full vehicle stack and stops at 4 seconds.
    try:
            while vis.Run() and system.GetChTime() < SIM_END:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                for _ in range(RENDER_EVERY):
                    time = system.GetChTime()
                    driver.Synchronize(time)
                    driver_inputs = driver.GetInputs()
                    terrain.Synchronize(time)
                    hmmwv.Synchronize(time, driver_inputs, terrain)
                    vis.Synchronize(time, driver_inputs)
                    driver.Advance(STEP_SIZE)
                    terrain.Advance(STEP_SIZE)
                    hmmwv.Advance(STEP_SIZE)
                    vis.Advance(STEP_SIZE)
                    if system.GetChTime() >= SIM_END:
                        break
                realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError) as exc:  # solver/API state failures
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:  # output path failures
        traceback.print_exc()
        raise
    finally:
        pass

    # === Post-processing ===


if __name__ == "__main__":
    main()
