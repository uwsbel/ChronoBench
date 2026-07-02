import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import random


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the SCM deformable terrain patch
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)

# Add randomly positioned boxes to the scene
NUM_BOXES = 10
MIN_DISTANCE_FROM_VEHICLE = 5.0
for _ in range(NUM_BOXES):
    while True:
        x = random.uniform(-40, 40)
        y = random.uniform(-40, 40)
        z = terrainHeight + 0.5
        pos = chrono.ChVector3d(x, y, z)
        if (pos - initLoc).Length() > MIN_DISTANCE_FROM_VEHICLE:
            break
    box = chrono.ChBodyEasyBox(2.0, 2.0, 1.0, 1000, True, True)
    box.SetPos(pos)
    box.SetFixed(True)
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    vehicle.GetSystem().AddBody(box)

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV with Sensors and Obstacles')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Add point lights to the scene
for i in range(4):
    light = chrono.ChPointLight()
    light.SetPosition(chrono.ChVector3d(20 * math.cos(i * math.pi/2), 20 * math.sin(i * math.pi/2), 10))
    light.SetI(chrono.ChColor(1.0, 1.0, 1.0))
    light.SetL(chrono.ChColor(1.0, 1.0, 1.0))
    light.SetQ(chrono.ChColor(1.0, 1.0, 1.0))
    vehicle.GetSystem().Add(light)

# Create sensor manager and camera sensor
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),  # parent body
    30,                        # update rate (Hz)
    chrono.ChFrameD(chrono.ChVector3d(0, 0, 2), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))),  # offset
    640, 480,                  # resolution
    60.0,                      # horizontal field of view
    0.01,                      # near clipping plane
    100.0                      # far clipping plane
)
camera.SetName("Vehicle Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0.1)
camera.PushFilter(sens.ChFilterVisualize(640, 480, "Vehicle Camera Feed"))
sensor_manager.AddSensor(camera)

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Update modules
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Update sensor manager
    sensor_manager.Update(time, step_size)
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.AdviceAdvance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)