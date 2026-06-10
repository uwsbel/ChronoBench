"""ARTcar vehicle simulation on rigid terrain.

This self-contained PyChrono 9.0 NSC vehicle model drives the catalog ARTcar
forward on a flat rigid road.  The vehicle uses the requested faster electric
drive settings, reduced tire rolling resistance, Bullet collision, a scripted
throttle driver, and an Irrlicht chase view to show the car accelerating.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named parameters make the edited vehicle settings explicit
STEP_SIZE = 0.002
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 60.0
TERRAIN_WIDTH = 8.0
INIT_Z = 0.125
TIRE_RADIUS = 0.085
WHEEL_Z_TOL = 0.02

MAX_MOTOR_VOLTAGE_RATIO = 0.26
STALL_TORQUE = 0.4
TIRE_ROLLING_RESISTANCE = 0.03


class FastARTcarDriver(veh.ChDriver):
    """Simple scored-core driver that applies steady throttle for acceleration."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetSteering(0.0)
        self.SetThrottle(0.0 if time < 0.25 else 0.85)
        self.SetBraking(1.0 if time < 0.25 else 0.0)


def main():
    # === Vehicle setup === catalog wrapper owns the NSC system and drivetrain
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, INIT_Z), chrono.QUNIT)
    )
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.SetTireStepSize(TIRE_STEP_SIZE)
    vehicle.SetMaxMotorVoltageRatio(MAX_MOTOR_VOLTAGE_RATIO)
    vehicle.SetStallTorque(STALL_TORQUE)
    vehicle.SetTireRollingResistance(TIRE_ROLLING_RESISTANCE)
    vehicle.Initialize()

    system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemNSC reused below
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    veh_obj = vehicle.GetVehicle()  # cache: vehicle subsystem reused for mass/spindles/vis
    chassis = vehicle.GetChassisBody()  # cache: chassis body queried every log row
    print("VEHICLE MASS: ", veh_obj.GetMass())

    # Wrapper-created components are explicit: system, chassis, suspension joints,
    # wheels, tires, drivetrain, driver, terrain, and vehicle-aware Irrlicht vis.
    spindle_positions = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_positions.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(pos.z for pos in spindle_positions) - TIRE_RADIUS
    assert wheel_bottom_z >= -WHEEL_Z_TOL, (
        f"ARTcar wheel bottom starts below terrain: {wheel_bottom_z:.4f} m"
    )

    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain === flat rigid road gives the faster ARTcar low rolling losses
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 80, 12)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Visualization === vehicle Irrlicht chase camera follows the moving car
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("ARTcar Faster Parameters")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.25), 3.5, 0.35)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(veh_obj)

    # === Driver === steady throttle demonstrates the increased motor authority
    driver = FastARTcarDriver(veh_obj)
    driver.Initialize()


    # === Main loop === synchronize driver, terrain, vehicle, and visualization
    realtime_timer = chrono.ChRealtimeStepTimer()
    frame = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                driver.Synchronize(time)
                driver_inputs = driver.GetInputs()  # cache: one input snapshot per step
                terrain.Synchronize(time)
                vehicle.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                vehicle.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)


                if system.GetChTime() >= SIM_END:
                    break

            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid vehicle state
        print(f"ARTcar simulation failed: {exc}")
        raise
    except (OSError, IOError) as exc:  # disk or permission error while recording
        print(f"ARTcar recording failed: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
