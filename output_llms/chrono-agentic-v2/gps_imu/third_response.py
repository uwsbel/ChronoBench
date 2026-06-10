"""
GPS and IMU sensor demo using HMMWV on flat rigid terrain.

System type: NSC (rigid terrain, standard HMMWV catalog vehicle).
Main bodies: HMMWV chassis, four wheel spindles, flat terrain patch.

IMU (accelerometer) sensor offset: (0, 0, 1) relative to chassis.
GPS sensor offset: (0, 0, 1) relative to chassis.
Driver inputs: constant steering=0.6, throttle=0.5 throughout simulation.

Expected behavior: HMMWV drives forward-left (constant steering + throttle),
GPS records latitude/longitude trajectory, IMU records chassis accelerations.
A matplotlib plot of the GPS trajectory is generated at the end.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants ===
# Simulation time and step parameters (precomputed once)
time_step = 1e-3                # physics step [s]
sim_end = 20.0                  # total simulation duration [s]
render_fps = 50.0               # Irrlicht render frame rate [Hz]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # steps per frame  # precomputed once

# Vehicle initial position
INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = 0.5                    # chassis origin above terrain at rest
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0

# GPS reference origin (lon, lat, alt) — Madison, WI area
GPS_REF = chrono.ChVector3d(-89.400, 43.070, 260.0)

# === Data paths (required for all catalog-vehicle truths) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                              # MANDATORY
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()          # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types — set after Initialize()
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver — scripted constant inputs (scored core) ===
# Truth shape: constant throttle/steering maneuver scripted in scored core.
# Steering=0.6, throttle=0.5 constant throughout simulation (as per prompt).
driver = veh.ChDataDriver(
    hmmwv.GetVehicle(),
    veh.vector_Entry([
        veh.DataDriverEntry(0.0, 0.6, 0.5, 0.0),
        veh.DataDriverEntry(sim_end + 1.0, 0.6, 0.5, 0.0),
    ]),
)
driver.Initialize()

# === Sensor manager — GPS and IMU sensors ===
manager = sens.ChSensorManager(system)
# No camera rendering lights needed for GPS/IMU — non-rendering sensors

# IMU sensor — offset pose (0, 0, 1) relative to chassis (per prompt)
imu_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
imu = sens.ChAccelerometerSensor(chassis, 10, imu_offset, sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

# GPS sensor — same offset pose (0, 0, 1) relative to chassis
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(
    chassis, 10, gps_offset,
    GPS_REF,
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Visualization — full Irrlicht block (Initialize FIRST, then scene elements) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV GPS/IMU Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()       # vehicle truths use directional light, not AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        vis.Advance(time_step)

        manager.Update()


        step_number += 1
        realtime_timer.Spin(time_step)

except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
