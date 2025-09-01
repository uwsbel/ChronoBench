import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math

# Set the Chrono data directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # Terrain height
terrainLength = 100.0  # Size in X direction
terrainWidth = 100.0   # Size in Y direction

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Logging step size and GPS data list
log_step_size = 0.1  # Log every 0.1 seconds
log_steps = int(log_step_size / step_size)
gps_data = []

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization types for vehicle parts
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChDriver()

# Initialize sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())

# Create an IMU sensor and add it to the manager
offset_pose = chrono.ChFrameD(chrono.ChVector3d(-8, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
imu = sens.ChIMUSensor(vehicle.GetChassisBody(),  # Body IMU is attached to
                       10,                        # Update rate in Hz
                       offset_pose,               # Offset pose
                       sens.ChNoiseNone())        # Noise model
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterIMUAccess())  # Corrected filter for IMU
manager.AddSensor(imu)

# Create a GPS sensor and add it to the manager
gps = sens.ChGPSSensor(vehicle.GetChassisBody(),                     # Body GPS is attached to
                       10,                        # Update rate in Hz
                       offset_pose,               # Offset pose
                       chrono.ChVector3d(-89.400, 43.070, 260.0),  # GPS reference point
                       sens.ChNoiseNone())        # Noise model
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# ---------------
# Simulation loop
# ---------------

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Set driver inputs based on simulation time
    if time < 6.0:
        throttle = 0.5
        steering = 0.1
        braking = 0.0
    else:
        throttle = 0.0
        steering = 0.0
        braking = 0.8

    driver.SetThrottle(throttle)
    driver.SetSteering(steering)
    driver.SetBraking(braking)

    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update sensors
    manager.Update()

    # Log GPS data
    if step_number % log_steps == 0:
        buffer = gps.GetMostRecentGPSBuffer()
        if buffer.HasData():
            data = buffer.GetGPSData()
            gps_data.append((data.Latitude, data.Longitude, data.Altitude))

    step_number += 1
    realtime_timer.Spin(step_size)

print("GPS Data: ", gps_data)