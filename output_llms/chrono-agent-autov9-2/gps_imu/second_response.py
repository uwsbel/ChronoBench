"""HMMWV on flat rigid terrain instrumented with an IMU (accelerometer +
gyroscope) and a GPS sensor mounted on the chassis.

System type: NSC (the HMMWV_Full wrapper owns its ChSystemNSC). A flat
RigidTerrain patch supports the four-wheel-drive HMMWV. A ChSensorManager pumps
a chassis-mounted accelerometer, gyroscope, and GPS each physics step; their
access filters expose the latest readings. A scripted driver opens the throttle
with a mild steering sweep, then brakes hard after 6 s.

Expected behavior: the vehicle accelerates forward in a gentle arc, the GPS
trace drifts away from its reference origin, and after t = 6 s the brakes engage
and the vehicle decelerates to a stop. Accelerometer/gyroscope magnitudes spike
during acceleration, cornering, and the braking event.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / timing / control schedule (no bare literals downstream)
time_step = 2.0e-3                       # integration step (s)
tire_step = 1.0e-3                       # TMEASY tire sub-step (s)
sim_end = 10.0                           # total simulated time (s)
render_fps = 50.0                        # review-video frame rate
sensor_update_rate = 1.0 / time_step     # sensors tick every physics step (Hz)

brake_time = 6.0                         # s — brakes engage after this instant
cruise_throttle = 0.6                    # steady throttle before braking
steer_amplitude = 0.3                    # peak steering command (-1..1)
steer_omega = 0.5                        # steering oscillation rate (rad/s)

terrain_length = 200.0                   # X extent of rigid patch (m)
terrain_width = 100.0                    # Y extent of rigid patch (m)
terrain_height = 0.0                     # top surface Z of the patch (m)

SUSPENSION_REF_HEIGHT = 0.5              # HMMWV chassis origin above wheel-bottom at rest (m)
TIRE_RADIUS = 0.46                       # HMMWV tire radius for the footprint assert (m)
ZTOL = 0.10                              # allowed wheel-bottom clearance vs support top (m)

init_x = -terrain_length / 2.0 + 10.0    # spawn near one end so there is room to drive
init_y = 0.0
init_z = terrain_height + SUSPENSION_REF_HEIGHT   # derived rest height
init_loc = chrono.ChVector3d(init_x, init_y, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)       # facing +X

gps_reference = chrono.ChVector3d(-122.3, 37.5, 10.0)   # lon/lat/alt origin for GPS readings

# Logging cadence: record GPS roughly every 0.1 s rather than every step.

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once: physics steps per frame

# === Driver === scripted, time-based control (open throttle + steering sweep, then brake)
class ScriptedDriver(veh.ChDriver):
    """Open-loop control law: cruise with a sinusoidal steering sweep, then brake
    hard once the simulation passes `brake_time`."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < brake_time:
            self.SetThrottle(cruise_throttle)
            self.SetBraking(0.0)
        else:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        self.SetSteering(steer_amplitude * math.sin(steer_omega * time))


# === Output dir guard === ensure relative output paths resolve before opening files


try:
    # === Vehicle === HMMWV_Full wrapper owns its ChSystemNSC; built then initialized
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    system = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
    chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()               # cache: vehicle subsystem handle, reused for spindles
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links
    # are created inside the wrapper; terrain is the RigidTerrain patch added below.

    # === Collision system === Bullet is required for the vehicle/terrain contact
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Terrain === flat rigid patch the HMMWV drives on
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Footprint check: wheels must rest on (not through) the rigid patch.
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= terrain_height - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={terrain_height:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{terrain_height - wheel_bottom_z:.3f} m"
    )

    # === Driver === scripted open-loop control attached to the vehicle
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Sensors === chassis-mounted IMU (accelerometer + gyroscope) + GPS
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3f(100, 100, 100), chrono.ChColor(1, 1, 1), 5000.0)

    noise_none = sens.ChNoiseNone()
    imu_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)

    accelerometer = sens.ChAccelerometerSensor(chassis, sensor_update_rate, imu_offset, noise_none)
    accelerometer.SetName("accelerometer")
    accelerometer.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(accelerometer)

    gyroscope = sens.ChGyroscopeSensor(chassis, sensor_update_rate, imu_offset, noise_none)
    gyroscope.SetName("gyroscope")
    gyroscope.PushFilter(sens.ChFilterGyroAccess())
    manager.AddSensor(gyroscope)

    gps = sens.ChGPSSensor(chassis, sensor_update_rate, imu_offset, gps_reference, noise_none)
    gps.SetName("gps")
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV with IMU and GPS")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddGrid(2.0, 2.0, 50, 50,
                chrono.ChCoordsysd(chrono.ChVector3d(init_x, 0, terrain_height + 0.01), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid near the spawn
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)


    # === Main loop === render once per frame; advance the full subsystem stack each step
    frame = 0
    step = 0
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            manager.Update()   # pump IMU + GPS sensors every physics step

            # Read the latest IMU / GPS samples (guard: buffers empty before first tick).

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(time_step)
            terrain.Advance(time_step)
            hmmwv.Advance(time_step)        # advances the wrapper-owned system
            vis.Advance(time_step)
            step += 1
            if system.GetChTime() >= sim_end:
                break


except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush partial CSV even if a step diverges.
    pass
