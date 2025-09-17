import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import sys

# Initialize core simulation system
system = chrono.ChSystemMulticore()
time_step = 0.01

# Create Irrlicht visualization
app = irr.ChIrrApp(system, 'Gator Simulation', irr.dimension2du(800, 600))
app.AddTypicalLights()
app.SetCamera(chrono.ChVectorD(0, 5, 10))  # Initial camera position
app.AssetSetMode(chrono.ChVisualizationAsset.Mode_RGB)
app.SetSymbolsScale(0.01)
app.SetTimestep(time_step)

# Create Gator vehicle
gator = veh.Gator()
gator.SetChassisVisualizationType(veh.VisualizationType.MESH)
gator.SetWheelVisualizationType(veh.VisualizationType.SIMPLE)
gator.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
gator.SetPowertrain(veh.SimplePowertrain())  # Default powertrain
gator.SetTireModel(veh.TireModel.SIMPLE)     # Default tire model

# Create terrain
terrain = veh.RigidTerrain(system, veh.materials.Concrete())
patch = veh.Patch()
patch.m_length = 100
patch.m_width = 100
patch.m_friction = 0.9
terrain.AddPatch(patch)
terrain.Initialize()
system.Add(terrain)

# Initialize vehicle with terrain
gator.Initialize(terrain)

# Add vehicle to system
system.Add(gator.GetChassis())

# Create driver
driver = veh.Driver()
driver.SetChassis(gator.GetChassis())
gator.SetDriver(driver)

# Initialize sensor manager
sensor_mgr = sens.SensorManager(system)
base_link = gator.GetChassis()
sensor_mgr.SetBaseLink(base_link)

# Add point light
light = sens.SensorLight()
light.SetName('point_light')
light.SetLightType(sens.SensorLightType.POINT)
light.SetPosition(chrono.ChVectorD(0, 0, 2))  # Position relative to base
light.SetIntensity(1000)
sensor_mgr.AddSensor(light)

# Add camera sensor
camera = sens.SensorCamera()
camera.SetName('front_camera')
camera.SetPosition(chrono.ChVectorD(0, 0, 1))  # Position on chassis
camera.SetOrientation(chrono.Q_from_Euler_123(chrono.ChVectorD(0, 0, 0)))  # Facing forward
camera.SetFOV(90)
camera.SetImageWidth(800)
camera.SetImageHeight(600)
camera.SetSensorRenderingType(sens.SensorRenderType.RGB)
sensor_mgr.AddSensor(camera)

sensor_mgr.Initialize()

# Simulation loop
frame = 0
while app.GetDevice().run():
    # Process driver inputs via keyboard
    key = app.GetKeyboard()
    steering = 0
    throttle = 0
    braking = 0

    if key == irr.KEYKEY_LEFT:
        steering = -1
    elif key == irr.KEYKEY_RIGHT:
        steering = 1
    if key == irr.KEYKEY_UP:
        throttle = 1
    if key == irr.KEYKEY_DOWN:
        braking = 1

    driver.SetInputs(steering, throttle, braking)

    # Update systems
    gator.Update()
    sensor_mgr.Update()

    # Advance simulation
    system.DoStepDynamics(time_step)

    # Render visualization
    app.BeginScene()
    app.DrawAll()
    terrain.Draw(app)
    app.EndScene()

    # Capture and save camera image
    filename = f'output/frame_{frame:05d}.jpg'
    sensor_mgr.AddCameraImage(camera.GetName(), filename)
    frame += 1

    # Exit condition
    if app.GetDevice().getKey(irr.KEY_ESCAPE):
        break

app.GetDevice().closeGraphicalWindow()