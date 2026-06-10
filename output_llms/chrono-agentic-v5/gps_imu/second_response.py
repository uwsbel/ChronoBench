"""GPS + IMU sensing on a driven HMMWV over rigid terrain (PyChrono 9.0.0, Irrlicht).

Models a full HMMWV wheeled vehicle (NSC contact) rolling on a flat RigidTerrain
patch. An OptiX ChSensorManager carries an accelerometer (IMU) and a GPS sensor,
both rigidly mounted on the chassis, each with its own access filter. The vehicle
is driven by a scripted open-loop maneuver: it accelerates with mild steering and
then brakes after 6 s. GPS coordinates are sampled at a fixed logging cadence
(log_step_size) via gps.GetMostRecentGPSBuffer().GetGPSData() and accumulated in a
gps_data list that is printed at the end of the run. Expected behavior: the HMMWV
drives forward along a gently curving path, decelerates after the brake onset, and
the logged GPS track shows the corresponding lat/lon drift away from the reference
origin.
"""

import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Parameters === geometry / timing / driver schedule (named, no bare literals)
time_step = 2e-3                       # integrator step (s)
sim_end = 10.0                         # total simulated time (s)
render_fps = 50.0                      # Irrlicht review-frame cadence
log_step_size = 0.1                    # GPS logging cadence (s) — frequency of data logging
brake_time = 6.0                       # s: braking is applied after this time
gps_reference = chrono.ChVector3d(-89.400, 43.070, 260.0)   # (lon, lat, alt) origin

init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

render_every = max(1, round(1.0 / (render_fps * time_step)))         # precomputed once
log_steps = max(1, round(log_step_size / time_step))                 # precomputed once: steps between GPS logs

# === Data paths === locate bundled Chrono + vehicle assets (truth-faithful pair)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV wrapper owns its own ChSystemNSC
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)     # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)            # rigid-terrain tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = vehicle.GetSystem()                                # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for contact
chassis_body = vehicle.GetChassisBody()                  # cache: main chassis rigid body, reused for sensors
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())  # report total vehicle mass

# === Terrain === flat rigid patch supporting the wheels
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         200.0, 200.0)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# === Sensors === IMU (accelerometer) + GPS rigidly mounted on the chassis
manager = sens.ChSensorManager(sys)
offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 1),
                              chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))

imu = sens.ChAccelerometerSensor(chassis_body, 10, offset_pose, sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())               # host access to accel buffer
manager.AddSensor(imu)

gps = sens.ChGPSSensor(chassis_body, 10, offset_pose, gps_reference, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())                 # host access to GPS buffer
manager.AddSensor(gps)

# === Visualization === full vehicle Irrlicht scene: window + sky + chase cam + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS + IMU")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === scripted open-loop schedule (throttle/steering, brake after 6 s)
driver_inputs = veh.DriverInputs()

# === Main loop === drive, pump sensors, log GPS at log_step_size cadence

gps_data = []          # accumulates logged GPS coordinates over the run
step_number = 0
frame = 0

os.makedirs("cam", exist_ok=True)   # guard against missing output dir
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        # Scripted driver schedule: accelerate with gentle steering, brake after 6 s.
        if time < brake_time:
            driver_inputs.m_throttle = 0.7
            driver_inputs.m_steering = 0.3 * math.sin(0.5 * time)
            driver_inputs.m_braking = 0.0
        else:
            driver_inputs.m_throttle = 0.0
            driver_inputs.m_steering = 0.0
            driver_inputs.m_braking = 0.8

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Synchronize the full subsystem stack, then advance it.
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        vis.Advance(time_step)
        manager.Update()                                 # pump sensors once per step

        # Log GPS coordinates at the configured cadence.
        if step_number % log_steps == 0:
            gps_buffer = gps.GetMostRecentGPSBuffer()    # may be empty before first GPS tick
            if gps_buffer.HasData():                     # guard: skip until the sensor has filled
                gps_pt = gps_buffer.GetGPSData()
                gps_data.append([gps_pt[0], gps_pt[1], gps_pt[2]])


        step_number += 1
except (RuntimeError, ValueError) as exc:                # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === report the accumulated GPS track
print("GPS Data: ", gps_data)
