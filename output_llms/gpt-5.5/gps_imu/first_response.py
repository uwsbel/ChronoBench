# hmmwv_imu_gps_irrlicht.py
#
# PyChrono simulation:
# - Full HMMWV vehicle
# - Rigid terrain
# - Irrlicht visualization and interactive driver
# - IMU/GPS sensors attached to chassis
# - Sensor processing/update each step
#
# Controls in Irrlicht driver window:
#   arrow keys / typical Chrono interactive driver bindings

import math
import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Chrono::Sensor is optional in some PyChrono installs.
try:
    import pychrono.sensor as sens
    HAS_CHRONO_SENSOR = True
except Exception:
    sens = None
    HAS_CHRONO_SENSOR = False


# -----------------------------------------------------------------------------
# Compatibility helpers for different PyChrono versions
# -----------------------------------------------------------------------------

def ChVector(x, y, z):
    if hasattr(chrono, "ChVector3d"):
        return chrono.ChVector3d(x, y, z)
    return chrono.ChVectorD(x, y, z)


def ChQuaternion(e0, e1, e2, e3):
    if hasattr(chrono, "ChQuaterniond"):
        return chrono.ChQuaterniond(e0, e1, e2, e3)
    return chrono.ChQuaternionD(e0, e1, e2, e3)


def QUNIT():
    return getattr(chrono, "QUNIT", ChQuaternion(1, 0, 0, 0))


def ChCoordsys(pos, rot):
    if hasattr(chrono, "ChCoordsysd"):
        return chrono.ChCoordsysd(pos, rot)
    return chrono.ChCoordsysD(pos, rot)


def ChFrame(pos, rot=None):
    if rot is None:
        rot = QUNIT()

    for cls_name in ["ChFramed", "ChFrameD", "ChFrame"]:
        if hasattr(chrono, cls_name):
            cls = getattr(chrono, cls_name)
            try:
                return cls(pos, rot)
            except Exception:
                pass

    # Fallback: many Chrono sensor constructors also accept ChCoordsys
    return ChCoordsys(pos, rot)


def vec_to_tuple(v):
    """Robustly convert a Chrono vector to a Python tuple."""
    vals = []
    for name, idx in [("x", 0), ("y", 1), ("z", 2)]:
        try:
            a = getattr(v, name)
            vals.append(a() if callable(a) else a)
        except Exception:
            vals.append(v[idx])
    return tuple(vals)


def body_linear_velocity(body):
    for method in ["GetPosDt", "GetLinVel", "GetVelocity"]:
        if hasattr(body, method):
            return vec_to_tuple(getattr(body, method)())
    return (0.0, 0.0, 0.0)


def body_angular_velocity(body):
    for method in ["GetAngVelParent", "GetWvel_par", "GetAngVelLocal"]:
        if hasattr(body, method):
            return vec_to_tuple(getattr(body, method)())
    return (0.0, 0.0, 0.0)


def create_contact_material(contact_method):
    if contact_method == chrono.ChContactMethod_NSC:
        if hasattr(chrono, "ChContactMaterialNSC"):
            mat = chrono.ChContactMaterialNSC()
        else:
            mat = chrono.ChMaterialSurfaceNSC()
    else:
        if hasattr(chrono, "ChContactMaterialSMC"):
            mat = chrono.ChContactMaterialSMC()
        else:
            mat = chrono.ChMaterialSurfaceSMC()

    mat.SetFriction(0.9)
    mat.SetRestitution(0.01)
    return mat


# -----------------------------------------------------------------------------
# Lightweight chassis sensor processor
# -----------------------------------------------------------------------------
# This class produces IMU/GPS-like processed data from the chassis state.
# If chrono.sensor is available, real Chrono sensor objects are also created and
# updated by ChSensorManager. This processor keeps the example robust across
# PyChrono builds and provides simple data for logging.

class ChassisIMUGPSProcessor:
    def __init__(self, chassis_body, gps_reference_lla):
        self.chassis = chassis_body

        # Reference LLA: latitude [deg], longitude [deg], altitude [m]
        self.lat0 = gps_reference_lla[0]
        self.lon0 = gps_reference_lla[1]
        self.alt0 = gps_reference_lla[2]

        self.last_time = None
        self.last_vel = None

        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.gps = (self.lat0, self.lon0, self.alt0)

    def update(self, time):
        pos = vec_to_tuple(self.chassis.GetPos())
        vel = body_linear_velocity(self.chassis)
        gyro = body_angular_velocity(self.chassis)

        if self.last_time is not None:
            dt = time - self.last_time
            if dt > 1e-12 and self.last_vel is not None:
                self.accel = (
                    (vel[0] - self.last_vel[0]) / dt,
                    (vel[1] - self.last_vel[1]) / dt,
                    (vel[2] - self.last_vel[2]) / dt,
                )

        self.gyro = gyro

        # Approximate local ENU meters to latitude/longitude/altitude.
        # Here Chrono X -> East, Y -> North, Z -> Up.
        earth_radius = 6378137.0
        east = pos[0]
        north = pos[1]
        up = pos[2]

        lat_rad = math.radians(self.lat0)
        lat = self.lat0 + math.degrees(north / earth_radius)
        lon = self.lon0 + math.degrees(east / (earth_radius * math.cos(lat_rad)))
        alt = self.alt0 + up

        self.gps = (lat, lon, alt)

        self.last_time = time
        self.last_vel = vel

        return {
            "imu": {
                "linear_acceleration_mps2": self.accel,
                "angular_velocity_radps": self.gyro,
            },
            "gps": {
                "latitude_deg": self.gps[0],
                "longitude_deg": self.gps[1],
                "altitude_m": self.gps[2],
            },
        }


# -----------------------------------------------------------------------------
# Optional Chrono sensor creation
# -----------------------------------------------------------------------------

def make_chrono_sensor_stack(system, chassis_body, gps_reference_lla):
    """
    Create Chrono Sensor IMU/GPS devices attached to the chassis.

    Returns:
        sensor_manager or None
    """
    if not HAS_CHRONO_SENSOR:
        print("[sensor] pychrono.sensor not available; using chassis-state IMU/GPS processor only.")
        return None

    try:
        manager = sens.ChSensorManager(system)

        imu_update_rate = 100.0
        gps_update_rate = 10.0
        sensor_offset_pose = ChFrame(ChVector(0.0, 0.0, 1.25), QUNIT())

        noise_none = sens.ChNoiseNone()

        # Accelerometer
        accel = sens.ChAccelerometerSensor(
            chassis_body,
            imu_update_rate,
            sensor_offset_pose,
            noise_none
        )
        accel.SetName("HMMWV chassis accelerometer")
        if hasattr(sens, "ChFilterAccelAccess"):
            accel.PushFilter(sens.ChFilterAccelAccess())
        manager.AddSensor(accel)

        # Gyroscope
        gyro = sens.ChGyroscopeSensor(
            chassis_body,
            imu_update_rate,
            sensor_offset_pose,
            noise_none
        )
        gyro.SetName("HMMWV chassis gyroscope")
        if hasattr(sens, "ChFilterGyroAccess"):
            gyro.PushFilter(sens.ChFilterGyroAccess())
        manager.AddSensor(gyro)

        # GPS
        if hasattr(sens, "ChGPSSensor"):
            gps_ref = ChVector(
                gps_reference_lla[0],
                gps_reference_lla[1],
                gps_reference_lla[2]
            )

            gps = sens.ChGPSSensor(
                chassis_body,
                gps_update_rate,
                sensor_offset_pose,
                gps_ref,
                noise_none
            )
            gps.SetName("HMMWV chassis GPS")
            if hasattr(sens, "ChFilterGPSAccess"):
                gps.PushFilter(sens.ChFilterGPSAccess())
            manager.AddSensor(gps)

        print("[sensor] Chrono Sensor IMU/GPS stack created and attached to chassis.")
        return manager

    except Exception as exc:
        print("[sensor] Could not create Chrono Sensor stack:", exc)
        print("[sensor] Continuing with chassis-state IMU/GPS processor.")
        return None


# -----------------------------------------------------------------------------
# Main simulation
# -----------------------------------------------------------------------------

def main():
    # -------------------------------------------------------------------------
    # Global Chrono data paths
    # -------------------------------------------------------------------------
    chrono_data = chrono.GetChronoDataPath()
    veh.SetDataPath(os.path.join(chrono_data, "vehicle") + "/")

    # -------------------------------------------------------------------------
    # Simulation parameters
    # -------------------------------------------------------------------------
    contact_method = chrono.ChContactMethod_NSC

    step_size = 2.0e-3
    tire_step_size = step_size

    render_fps = 50
    render_step_size = 1.0 / render_fps

    end_time = 120.0

    sensor_print_period = 0.25

    init_loc = ChVector(0.0, 0.0, 1.0)
    init_rot = QUNIT()

    # Reference GPS point, latitude [deg], longitude [deg], altitude [m]
    gps_reference_lla = (43.073268, -89.400636, 260.0)

    # -------------------------------------------------------------------------
    # HMMWV full vehicle setup
    # -------------------------------------------------------------------------
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ChCoordsys(init_loc, init_rot))
    hmmwv.SetInitFwdVel(0.0)

    # Vehicle subsystem choices
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size)

    hmmwv.Initialize()

    system = hmmwv.GetSystem()

    # Visualization detail
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    chassis_body = hmmwv.GetChassisBody()

    try:
        vehicle_mass = hmmwv.GetVehicle().GetMass()
    except Exception:
        vehicle_mass = chassis_body.GetMass()

    print("HMMWV vehicle mass: {:.3f} kg".format(vehicle_mass))

    # -------------------------------------------------------------------------
    # Terrain
    # -------------------------------------------------------------------------
    terrain = veh.RigidTerrain(system)

    terrain_mat = create_contact_material(contact_method)

    terrain_patch = terrain.AddPatch(
        terrain_mat,
        ChCoordsys(ChVector(0.0, 0.0, 0.0), QUNIT()),
        300.0,
        300.0
    )
    terrain_patch.SetColor(chrono.ChColor(0.45, 0.50, 0.35))

    try:
        terrain_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    except Exception:
        pass

    terrain.Initialize()

    # -------------------------------------------------------------------------
    # Irrlicht visual system
    # -------------------------------------------------------------------------
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("PyChrono HMMWV with IMU and GPS Sensors")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(ChVector(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()

    try:
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    except Exception:
        pass

    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # -------------------------------------------------------------------------
    # Interactive driver
    # -------------------------------------------------------------------------
    driver = veh.ChInteractiveDriverIRR(vis)

    steering_time = 1.0
    throttle_time = 1.0
    braking_time = 0.3

    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # -------------------------------------------------------------------------
    # Sensors
    # -------------------------------------------------------------------------
    sensor_manager = make_chrono_sensor_stack(system, chassis_body, gps_reference_lla)
    sensor_processor = ChassisIMUGPSProcessor(chassis_body, gps_reference_lla)

    # -------------------------------------------------------------------------
    # Simulation loop
    # -------------------------------------------------------------------------
    realtime_timer = chrono.ChRealtimeStepTimer()

    render_time = 0.0
    last_sensor_print = -1.0

    print("Starting simulation loop...")

    while vis.Run():
        time = system.GetChTime()

        if time >= end_time:
            break

        # Render scene
        if time >= render_time:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_time += render_step_size

        # Driver inputs
        driver_inputs = driver.GetInputs()

        # Synchronize modules
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        # Update Chrono Sensor manager, if available
        if sensor_manager is not None:
            sensor_manager.Update()

        # Process IMU/GPS data from chassis state
        sensor_data = sensor_processor.update(system.GetChTime())

        # Output processed sensor data at a lower rate
        if time - last_sensor_print >= sensor_print_period:
            imu = sensor_data["imu"]
            gps = sensor_data["gps"]

            print(
                "t = {:8.3f} s | "
                "IMU accel = ({: .3f}, {: .3f}, {: .3f}) m/s^2 | "
                "gyro = ({: .3f}, {: .3f}, {: .3f}) rad/s | "
                "GPS = ({:.7f}, {:.7f}, {:.2f} m)".format(
                    time,
                    imu["linear_acceleration_mps2"][0],
                    imu["linear_acceleration_mps2"][1],
                    imu["linear_acceleration_mps2"][2],
                    imu["angular_velocity_radps"][0],
                    imu["angular_velocity_radps"][1],
                    imu["angular_velocity_radps"][2],
                    gps["latitude_deg"],
                    gps["longitude_deg"],
                    gps["altitude_m"],
                )
            )

            last_sensor_print = time

        realtime_timer.Spin(step_size)

    print("Simulation finished.")


if __name__ == "__main__":
    main()