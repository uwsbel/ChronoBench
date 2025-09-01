import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sensor
import math
import random
import cv2

# Set Chrono data path
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
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create and initialize SCM deformable terrain
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# Add random boxes to the scene
num_boxes = 10
for _ in range(num_boxes):
    # Random position avoiding the vehicle area
    x = random.uniform(-50, 50)
    y = random.uniform(-50, 50)
    z = 0.5  # Height above terrain
    
    # Ensure boxes are not too close to the vehicle
    if abs(x + 8) < 5 or abs(y) < 5:
        continue
    
    # Random size and color
    size = random.uniform(1, 3)
    color = chrono.ChColor(random.uniform(0,1), random.uniform(0,1), random.uniform(0,1))
    
    # Create and add box
    box = chrono.ChBodyEasyBox(size, size, size, 1000)
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetMaterialSurface(chrono.ChMaterialSurface())
    box.GetVisualShape().SetMaterialColor(color)
    vehicle.GetSystem().Add(box)

# Create sensor manager
sensor_manager = sensor.ChSensorManager(vehicle.GetSystem())

# Add point lights
light1 = sensor.ChPointLight()
light1.SetPosition(chrono.ChVector3d(10, 10, 10))
light1.SetIntensity(100)
sensor_manager.AddSensor(light1)

light2 = sensor.ChPointLight()
light2.SetPosition(chrono.ChVector3d(-10, -10, 10))
light2.SetIntensity(100)
sensor_manager.AddSensor(light2)

# Create camera sensor
camera = sensor.ChCameraSensor()
camera.SetResolution(640, 480)
camera.SetFieldOfView(60)
camera.SetPosition(chrono.ChVector3d(0, 0, 1.71))  # Attached to vehicle chassis
camera.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))
camera.SetSensorManager(sensor_manager)

# Create filter for camera visualization
class CameraFilter(sensor.ChFilter):
    def Process(self, data):
        img = data.GetImage()
        cv2.imshow('Camera View', img)
        cv2.waitKey(1)

filter = CameraFilter()
camera.AddFilter(filter)

# Connect filter to camera
camera.Connect(filter)

# Simulation loop
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

    step_number += 1
    realtime_timer.Spin(step_size)

# Cleanup
cv2.destroyAllWindows()