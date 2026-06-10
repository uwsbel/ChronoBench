import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math

# Set the Chrono data directories
chrono.SetChronoDataPath(chrono.GetChronoDataPath())               # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')           # locate vehicle data files

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)                             # chassis origin spawn (on terrain)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                        # identity orientation

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH                             # mesh visuals for the HMMWV

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE                   # no chassis collision

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY                             # TMEASY tires on rigid terrain

# Rigid terrain parameters
terrainHeight = 0                                                 # terrain height
terrainLength = 100.0                                             # size in X direction
terrainWidth = 100.0                                              # size in Y direction

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)                   # chase-camera track point

# Contact method
contact_method = chrono.ChContactMethod_NSC                      # NSC for rigid terrain

# Simulation step sizes
step_size = 1e-3                                                  # integration step
tire_step_size = step_size                                       # tire substep equals main step

# Time interval between two render frames
render_step_size = 1.0 / 50                                      # FPS = 50
log_step_size = 1.0 / 20                                         # frequency of GPS data logging

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()                                       # full HMMWV model
vehicle.SetContactMethod(contact_method)                         # NSC contact
vehicle.SetChassisCollisionType(chassis_collision_type)          # no chassis collision
vehicle.SetChassisFixed(False)                                   # chassis must be free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))    # spawn pose
vehicle.SetTireType(tire_model)                                  # TMEASY tire
vehicle.SetTireStepSize(tire_step_size)                          # tire integration step
vehicle.Initialize()                                             # build the vehicle

# Set visualization types for vehicle parts
vehicle.SetChassisVisualizationType(vis_type)                    # chassis mesh
vehicle.SetSuspensionVisualizationType(vis_type)                 # suspension mesh
vehicle.SetSteeringVisualizationType(vis_type)                   # steering mesh
vehicle.SetWheelVisualizationType(vis_type)                      # wheel mesh
vehicle.SetTireVisualizationType(vis_type)                       # tire mesh

# Set collision system type (required for contact/terrain scenes)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()                        # rigid-terrain contact material
patch_mat.SetFriction(0.9)                                       # friction
patch_mat.SetRestitution(0.01)                                   # restitution
terrain = veh.RigidTerrain(vehicle.GetSystem())                  # rigid terrain on the vehicle system
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)             # flat ground patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # ground texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                    # ground tint
terrain.Initialize()                                             # finalize terrain

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                 # vehicle Irrlicht window
vis.SetWindowTitle('HMMWV Demo')                                 # window title
vis.SetWindowSize(1280, 1024)                                    # window size
vis.SetChaseCamera(trackPoint, 6.0, 0.5)                         # chase camera behind chassis
vis.Initialize()                                                 # Initialize FIRST, then add scene elements
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # logo
vis.AddLightDirectional()                                        # vehicle demos use a directional light
vis.AddSkyBox()                                                  # sky box
vis.AttachVehicle(vehicle.GetVehicle())                          # bind chassis/wheel visuals

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)                         # interactive driver bound to vis

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0                                              # time to go 0 -> +1 steering
throttle_time = 1.0                                              # time to go 0 -> +1 throttle
braking_time = 0.3                                               # time to go 0 -> +1 braking
driver.SetSteeringDelta(render_step_size / steering_time)        # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)        # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)          # braking rate
driver.Initialize()                                              # build the driver

# Initialize sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())              # oversees all sensors

# Create an IMU sensor and add it to the manager
offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))  # sensor offset on chassis
imu = sens.ChAccelerometerSensor(vehicle.GetChassisBody(),       # body the IMU is attached to
                                 10,                             # update rate in Hz
                                 offset_pose,                    # offset pose
                                 sens.ChNoiseNone())             # noise model
imu.SetName("IMU Sensor")                                       # sensor name
imu.SetLag(0)                                                    # no lag
imu.SetCollectionWindow(0)                                       # instantaneous collection
imu.PushFilter(sens.ChFilterAccelAccess())                      # host access to IMU data
manager.AddSensor(imu)                                           # register the IMU

# Create a GPS sensor and add it to the manager
gps = sens.ChGPSSensor(vehicle.GetChassisBody(),                 # body the GPS is attached to
                       10,                                      # update rate in Hz
                       offset_pose,                             # offset pose
                       chrono.ChVector3d(-89.400, 43.070, 260.0),  # GPS reference (lat, lon, alt)
                       sens.ChNoiseNone())                      # noise model
gps.SetName("GPS Sensor")                                       # sensor name
gps.SetLag(0)                                                    # no lag
gps.SetCollectionWindow(0)                                       # instantaneous collection
gps.PushFilter(sens.ChFilterGPSAccess())                        # host access to GPS data
manager.AddSensor(gps)                                           # register the GPS

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())         # report total vehicle mass

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)          # steps per render frame
log_steps = math.ceil(log_step_size / step_size)                # steps per GPS log sample
realtime_timer = chrono.ChRealtimeStepTimer()                   # wall-clock pacing
step_number = 0                                                 # physics step counter
render_frame = 0                                                # rendered-frame counter


gps_data = []                                                  # collected GPS samples (scored sensor output)
# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()                     # current sim time

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    if step_number % log_steps == 0:
        # get most recent GPS data
        gps_coor = gps.GetMostRecentGPSBuffer().GetGPSData()   # latest GPS fix
        gps_data.append([gps_coor[0], gps_coor[1], gps_coor[2]])

    # Set driver inputs: constant steering 0.6, throttle 0.5
    driver.SetSteering(0.6)                                    # constant steering
    driver.SetThrottle(0.5)                                    # constant throttle
    driver_inputs = driver.GetInputs()                        # gather inputs

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)                                  # driver sync
    terrain.Synchronize(time)                                 # terrain sync
    vehicle.Synchronize(time, driver_inputs, terrain)         # vehicle sync
    vis.Synchronize(time, driver_inputs)                      # vis sync

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)                                 # advance driver
    terrain.Advance(step_size)                                # advance terrain
    vehicle.Advance(step_size)                                # advance vehicle (steps the system)
    vis.Advance(step_size)                                    # advance vis

    # Update sensor manager in each step
    manager.Update()                                          # pump sensors once per step

    step_number += 1                                          # next step
    realtime_timer.Spin(step_size)                            # pace to real time


print("GPS Data: ", gps_data)                                # report collected GPS trajectory
