"""BMW E90 sedan driving on a rigid highway terrain.

This NSC vehicle simulation initializes a movable sedan on a highway surface,
uses Bullet collision for tire-terrain contact, and applies a PID speed
controller with a five-second steering response time to hold a reference speed.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants === physics and controller values kept visible for review
STEP_SIZE = 5.0e-4
SIM_END = 4.0
RENDER_STEP_SIZE = 1.0 / 100.0
RENDER_EVERY = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

INIT_LOC = chrono.ChVector3d(-80.0, -2.0, 0.42)
INIT_YAW = 0.0
INIT_ROT = chrono.QuatFromAngleAxis(INIT_YAW, chrono.ChVector3d(0, 0, 1))
TIRE_RADIUS = 0.31
WHEEL_Z_TOL = 0.12

REFERENCE_SPEED = 14.0
PID_KP = 0.08
PID_KI = 0.018
PID_KD = 0.006
STEERING_RESPONSE_TIME = 5.0
TARGET_STEERING = 0.0
THROTTLE_LIMIT = 1.0
BRAKE_LIMIT = 0.7

TERRAIN_LENGTH = 220.0
TERRAIN_WIDTH = 16.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 0.75)


# === Driver === closed-loop speed control requested by the task
class PIDSpeedDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)
        self.vehicle_ref = vehicle  # cache: Python handle reused by the PID loop
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = 0.0
        self.steering = 0.0

    def Synchronize(self, time):
        speed = self.vehicle_ref.GetSpeed()
        dt = max(time - self.last_time, STEP_SIZE)
        error = REFERENCE_SPEED - speed
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        effort = PID_KP * error + PID_KI * self.integral + PID_KD * derivative

        if effort >= 0.0:
            throttle = min(THROTTLE_LIMIT, effort)
            braking = 0.0
        else:
            throttle = 0.0
            braking = min(BRAKE_LIMIT, -effort)

        steer_alpha = min(1.0, dt / STEERING_RESPONSE_TIME)
        self.steering += (TARGET_STEERING - self.steering) * steer_alpha

        self.SetThrottle(throttle)
        self.SetBraking(braking)
        self.SetSteering(self.steering)

        self.last_error = error
        self.last_time = time


def main():
    # === Vehicle and system === BMW wrapper owns the ChSystemNSC
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    vehicle = veh.BMW_E90()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    vehicle.SetTireType(veh.TireModelType_TMEASY)  # BMW_E90-supported highway tire contact
    vehicle.SetTireStepSize(STEP_SIZE)
    vehicle.Initialize()

    system = vehicle.GetSystem()  # cache: wrapper-owned system reused throughout
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chassis = vehicle.GetChassisBody()  # cache: chassis handle reused by terrain and logging
    veh_obj = vehicle.GetVehicle()  # cache: low-level vehicle handle for mass and spindle checks
    print("VEHICLE MASS: ", veh_obj.GetMass())
    # wrapper components: chassis, suspension, steering, wheels, tires, and joints are created by BMW_E90

    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain === rigid highway strip provides the requested road surface
    terrain_mat = chrono.ChContactMaterialNSC()
    terrain_mat.SetFriction(TERRAIN_FRICTION)
    terrain_mat.SetRestitution(TERRAIN_RESTITUTION)
    terrain = veh.RigidTerrain(system)
    terrain_patch = terrain.AddPatch(terrain_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    terrain_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 120, 8)
    terrain.Initialize()

    spindle_positions = []
    for axle_index in range(veh_obj.GetNumberAxles()):
        spindle_positions.append(veh_obj.GetSpindlePos(axle_index, veh.LEFT))
        spindle_positions.append(veh_obj.GetSpindlePos(axle_index, veh.RIGHT))
    wheel_bottom_z = min(pos.z for pos in spindle_positions) - TIRE_RADIUS
    terrain_height = terrain.GetHeight(INIT_LOC)
    assert wheel_bottom_z >= terrain_height - WHEEL_Z_TOL, (
        f"vehicle sinks into highway mesh: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain height z={terrain_height:.3f}"
    )

    # === Visualization === vehicle-specific Irrlicht window and chase camera
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("BMW E90 Sedan Highway PID Speed Control")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_POINT, 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(veh_obj)

    # === Controls === PID driver regulates speed from the vehicle state
    driver = PIDSpeedDriver(veh_obj)
    driver.Initialize()


    # === Main loop === render-cadence loop with full vehicle subsystem stepping
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    try:

        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()  # cache: one input snapshot per step


                driver.Synchronize(time)
                terrain.Synchronize(time)
                vehicle.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                vehicle.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)

                step_number += 1
                realtime_timer.Spin(STEP_SIZE)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:  # solver divergence or invalid vehicle state
        raise
    except (OSError, IOError) as exc:  # disk or frame write failure
        raise
    finally:
        pass

    if step_number <= 0:
        raise RuntimeError("simulation did not advance any vehicle steps")


if __name__ == "__main__":
    main()
