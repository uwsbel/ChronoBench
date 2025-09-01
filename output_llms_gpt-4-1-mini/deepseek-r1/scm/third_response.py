import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as csensor
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

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

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

# Create and initialize the HMMWV vehicle
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

# Create the SCM deformable terrain
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)

# Add randomly positioned boxes
num_boxes = 20
box_size = chrono.ChVector3d(0.5, 0.5, 0.5)
vehicle_radius = 3.0  # Minimum distance from vehicle's initial position

for _ in range(num_boxes):
    while True:
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        z = box_size.z / 2
        pos = chrono.ChVector3d(x, y, z)
        init_veh_pos = chrono.ChVector3d(-8, 0, 0.6)
        dist = (pos - init_veh_pos).Length()
        if dist > vehicle_radius:
            break
    
    box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 1000)
    box.SetPos(pos)
    box.SetFixed(True)
    box.SetCollide(True)
    box.GetVisualShape(0).SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
    vehicle.GetSystem().AddBody(box)

# Create Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create sensor manager
sensor_manager = csensor.ChSensorManager(vehicle.GetSystem())
sensor_manager.SetKeyframeSizeFromTimeStep(step_size, 1)

# Add point lights
sensor_manager.scene.AddPointLight(chrono.ChVector3d(10, 10, 20), chrono.ChColor(1, 1, 1), 500.0)
sensor_manager.scene.AddPointLight(chrono.ChVector3d(-10, -10, 20), chrono.ChColor(1, 1, 1), 500.0)

# Create and attach camera sensor
camera_body = vehicle.GetChassisBody()
camera_pos = chrono.ChVector3d(-2, 0, 1.5)
camera_rot = chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))
camera_frame = chrono.ChFrameD(camera_pos, camera_rot)

camera = csensor.ChCameraSensor(
    camera_body,
    30,
    camera_frame,
    1280,
    720,
    math.radians(90)
)
camera.SetName("Vehicle Camera")
camera.PushFilter(csensor.ChFilterRGBA8())
camera.PushFilter(csensor.ChFilterVisualize(1280, 720, "Camera Feed"))
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

# Simulation loop setup
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

    # Get driver inputs
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
    sensor_manager.Update()

    step_number += 1
    realtime_timer.Spin(step_size)