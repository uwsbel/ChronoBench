import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization and collision types
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50

# Create and initialize the MAN vehicle
vehicle = veh.MAN_10t()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Set collision system type after initialization
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystemType.BULLET)

# Visualization settings
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Create terrain with grass texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Add random boxes to the terrain
for _ in range(5):
    x = np.random.uniform(-40, 40)
    y = np.random.uniform(-40, 40)
    z = 0.5  # Slightly above terrain
    box = chrono.ChBodyEasyBox(2, 2, 1, 2000, True, True)  # Steel box
    box.SetPos(chrono.ChVectorD(x, y, z))
    box_mat = chrono.ChMaterialSurfaceNSC()
    box_mat.SetFriction(0.9)
    box.SetMaterialSurface(box_mat)
    vehicle.GetSystem().AddBody(box)

# Initialize Irrlicht interface
vis = irr.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# Create sensor manager and lidar
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetBody(vehicle.GetChassis())  # Attach to vehicle's chassis
lidar.SetPosition(chrono.ChVectorD(0, 0, 1.5))  # Position on the chassis
lidar.SetDirection(chrono.ChVectorD(0, 1, 0))  # Forward direction (Y-axis)
lidar.SetRange(50)
lidar.SetOpeningAngle(chrono.CH_C_PI / 4)
lidar.SetHorizontalResolution(0.1)
lidar.SetVerticalResolution(0.1)
lidar.SetFovHorizontal(chrono.CH_C_PI / 2)
lidar.SetFovVertical(chrono.CH_C_PI / 4)
lidar.SetSamplingDistance(0.1)
sensor_manager.AddSensor(lidar)

# Driver setup
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize(vehicle)  # Pass the vehicle to Initialize

# Simulation control
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    driver_inputs = driver.GetInputs()
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    # Update sensor manager
    sensor_manager.Update()
    
    step_number += 1
    realtime_timer.Spin(step_size)