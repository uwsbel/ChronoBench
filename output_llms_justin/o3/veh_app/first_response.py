################################################################################
# PyChrono demo
#
#  - Gator vehicle with customized visualization
#  - Rigid terrain
#  - Interactive Irrlicht-based driver
#  - Sensor manager with point lights and a chassis-mounted camera
#  - Real-time simulation loop, synchronizing and advancing all modules
################################################################################
#
# PREREQUISITES
#   * PyChrono compiled with Irrlicht     (for interactive driver / real-time GUI)
#   * PyChrono compiled with Sensor module (for camera and lights)
#   * The “vehicle” data directory on the Chrono data path
################################################################################

import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


# ------------------------------------------------------------------------------
# GLOBAL SETTINGS
# ------------------------------------------------------------------------------

step_size            = 1e-3          # simulation step size
render_step           = 0.03         # render every N seconds
end_time              = 10.0         # run for N seconds
contact_method        = chrono.ChContactMethod_NSC
data_path             = chrono.GetChronoDataPath()
vehicle_data_path     = veh.GetDataPath()

# Seed Chrono's random number generator (optional, good for sensors with noise)
chrono.ChRandomGenerator.SetSeed(12345)

# ------------------------------------------------------------------------------
# 1. CREATE THE PHYSICAL SYSTEM
# ------------------------------------------------------------------------------

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# ------------------------------------------------------------------------------
# 2. GATOR VEHICLE INITIALIZATION
# ------------------------------------------------------------------------------

# Initial position & orientation of the vehicle
init_loc  = chrono.ChVectorD(0, 0, 0.4)
init_rot  = chrono.ChQuaternionD(1, 0, 0, 0)          # no rotation

gator = veh.Gator(system)
gator.SetContactMethod(contact_method)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))

# Visualization types ----------------------------------------------------------
gator.SetChassisVisualizationType(veh.VisualizationType.MESH)
gator.SetWheelVisualizationType  (veh.VisualizationType.PRIMITIVES)
gator.SetTireVisualizationType   (veh.VisualizationType.SPRINGS)   # just to show variety
gator.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
gator.Initialize()

# ------------------------------------------------------------------------------
# 3. RIGID TERRAIN
# ------------------------------------------------------------------------------

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_size = 400
terrain_patch = terrain.AddPatch(patch_mat,
                                 chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT),
                                 patch_size, patch_size)
terrain_patch.SetColor(chrono.ChColor(0.4, 0.5, 0.4))
terrain_patch.SetTexture(data_path + "terrain/textures/grass.jpg", 10, 10)
terrain.Initialize()

# ------------------------------------------------------------------------------
# 4. INTERACTIVE DRIVER (IRRLICHT GUI)
# ------------------------------------------------------------------------------

app = veh.ChVehicleIrrApp(gator, "Gator with Sensors", chrono.dimension(1280, 720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalCamera(chrono.vectorD(0.0, 2.0, 1.2), chrono.vectorD(0, 0, 0.4))
app.AddLightWithShadow(chrono.vectorD(5,5,5), chrono.vectorD(0,0,0), 15, 4, 10, 60)
app.AddLightDirectional()

# Bind and update asset system once
chrono.ChAssetLevel().BindAll(system)
chrono.ChAssetLevel().UpdateAll(system)

driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.03)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.2)
driver.Initialize()

# ------------------------------------------------------------------------------
# 5. SENSOR MANAGER, LIGHTS, CAMERA
# ------------------------------------------------------------------------------

# 5.1 Manager
sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetKeyframeSizeFromTime(0.1)           # how long to keep data in buffer

# 5.2 Point light(s)
pl_1 = sens.ChPointLight(chrono.ChVectorF(10, 10, 10),      # position
                         chrono.ChColor(1.0, 1.0, 1.0),     # color
                         400.0)                             # intensity
sensor_manager.AddSensor(pl_1)

pl_2 = sens.ChPointLight(chrono.ChVectorF(-10, -10, 10),
                         chrono.ChColor(1.0, 0.8, 0.8),
                         200.0)
sensor_manager.AddSensor(pl_2)

# 5.3 Camera attached to the vehicle chassis
cam_update_rate  = 30.0                                     # FPS
cam_resolution   = sens.ChVector2i(1280, 720)
cam_fov          = math.radians(70)

# relative placement on chassis
cam_offset_pose  = chrono.ChFrameD(chrono.ChVectorD(0.5, 0.0, 1.2),
                                   chrono.Q_from_AngAxis(-math.pi/6, chrono.VECT_Y))

camera = sens.ChCameraSensor(gator.GetChassis(),             # body to which the sensor is attached
                             cam_update_rate,                # update rate (Hz)
                             cam_offset_pose,                # offset pose
                             cam_resolution,                 # resolution
                             cam_fov)                        # vertical FOV (rad)

# Add sensor post-processing filters
camera.PushFilter(sens.ChFilterRGBA8Access())                # Access filter required before save
camera.PushFilter(sens.ChFilterSave("output/cam/"))          # Will save PNGs in this folder

sensor_manager.AddSensor(camera)

# ------------------------------------------------------------------------------
# 6. SIMULATION LOOP
# ------------------------------------------------------------------------------

realtime_timer      = chrono.ChRealtimeStepTimer()
realtime_timer.SetDesiredRealtimeRate(1.0)

time_last_render    = 0.0
step_number         = 0

print("Simulation start -----------------------------------------------")
while (system.GetChTime() < end_time):

    # Render Irrlicht view at the desired FPS
    if system.GetChTime() - time_last_render >= render_step:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()
        time_last_render = system.GetChTime()

    # Get driver inputs and pass to the vehicle
    driver_inputs = driver.GetInputs()
    gator.GetVehicle().Synchronize(system.GetChTime(),
                                   driver_inputs,
                                   terrain)

    # Synchronize terrain (includes collision set-up etc.)
    terrain.Synchronize(system.GetChTime())

    # Update modules
    driver.Advance(step_size)
    gator.GetVehicle().Advance(step_size)
    terrain.Advance(step_size)

    # Update the sensor manager (handles all sensors, launches GPU kernels, saves, etc.)
    sensor_manager.Update()

    # Advance the simulation state
    system.DoStepDynamics(step_size)

    # Real-time pacing
    realtime_timer.Spin(step_size)

    step_number += 1

print("Simulation finished.")