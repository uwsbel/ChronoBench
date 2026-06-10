"""Kraz tractor-trailer double lane change on rigid terrain.

This PyChrono NSC vehicle simulation starts a Kraz tractor-trailer at
(-15, 0, 0.5), follows it with an Irrlicht chase camera aimed at the
front track point, and drives a time-based double lane change maneuver.
The Kraz wrapper owns the chassis, suspension, steering, tires, trailer,
and contact system; a flat rigid terrain patch provides road contact.
"""

import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants === simulation timing, vehicle start, camera, and terrain sizes
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 5.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 12.0
RENDER_FPS = 10.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

INIT_LOC = chrono.ChVector3d(-15.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
INIT_FWD_VEL = 0.0
CHASE_TRACK_POINT = chrono.ChVector3d(3.0, 0.0, 2.1)
CHASE_DISTANCE = 25.0
CHASE_HEIGHT = 10.5

TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 40.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
WHEEL_RADIUS_EST = 0.50
SPAWN_Z_TOL = 0.20


# === Driver === time-based control applies throttle and double lane change steering
class DoubleLaneChangeDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            steering = 0.0
            throttle = 0.0
            braking = 0.0
        elif time < 1.2:
            steering = 0.0
            throttle = 0.70
            braking = 0.0
        elif time < 2.2:
            steering = 0.85
            throttle = 0.70
            braking = 0.0
        elif time < 4.6:
            steering = -1.0
            throttle = 0.62
            braking = 0.0
        elif time < 7.0:
            steering = 1.0
            throttle = 0.62
            braking = 0.0
        elif time < 8.8:
            steering = -0.55
            throttle = 0.52
            braking = 0.0
        else:
            steering = 0.0
            throttle = 0.48
            braking = 0.0

        self.SetSteering(steering)
        self.SetThrottle(throttle)
        self.SetBraking(braking)


# === Vehicle and terrain === wrapper builds Kraz bodies, joints, tires, and trailer
def build_simulation():
    kraz = veh.Kraz()
    kraz.SetContactMethod(chrono.ChContactMethod_NSC)
    kraz.SetChassisCollisionType(veh.CollisionType_NONE)
    kraz.SetChassisFixed(False)
    kraz.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    kraz.SetInitFwdVel(INIT_FWD_VEL)
    kraz.SetTireStepSize(TIRE_STEP_SIZE)
    kraz.Initialize()

    system = kraz.GetSystem()  # cache: wrapper-owned ChSystemNSC reused throughout
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    tractor = kraz.GetTractor()  # cache: tractor vehicle handle for driver, mass, spindles
    chassis = tractor.GetChassisBody()  # cache: chassis body used for validation/logging
    print("VEHICLE MASS: ", tractor.GetMass())
    # wrapper components: system, tractor chassis, trailer bodies, suspension, steering,
    # tires, and articulation joints are created inside veh.Kraz.

    kraz.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
    kraz.SetSuspensionVisualizationType(
        veh.VisualizationType_PRIMITIVES,
        veh.VisualizationType_PRIMITIVES,
    )
    kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
    kraz.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    spindle_positions = []
    for axle_index in range(tractor.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_positions.append(tractor.GetSpindlePos(axle_index, side))
    wheel_bottom_z = min(pos.z for pos in spindle_positions) - WHEEL_RADIUS_EST
    assert wheel_bottom_z >= -SPAWN_Z_TOL, (
        f"vehicle spawn is below terrain: wheel bottom z={wheel_bottom_z:.3f}"
    )

    return kraz, system, tractor, chassis, terrain


# === Visualization === vehicle-aware Irrlicht chase camera with sky and directional light
def build_visualization(kraz):
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Kraz Double Lane Change")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_TRACK_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(kraz.GetTractor())
    return vis


# === Main loop === synchronize and advance driver, terrain, Kraz, and visualization
def run():
    kraz, system, tractor, chassis, terrain = build_simulation()
    vis = build_visualization(kraz)
    driver = DoubleLaneChangeDriver(tractor)
    driver.Initialize()

    frame = 0
    step_number = 0
    realtime_timer = chrono.ChRealtimeStepTimer()

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
                kraz.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)


                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                kraz.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)
                step_number += 1
                realtime_timer.Spin(STEP_SIZE)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:  # solver/API failure or invalid state
        print(f"simulation failed: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    try:
        run()
    except (OSError, IOError) as exc:  # output directory or CSV file failure
        print(f"file output failed: {exc}")
        raise
