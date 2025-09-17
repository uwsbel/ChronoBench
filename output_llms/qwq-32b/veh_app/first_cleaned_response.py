import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import sys


system = chrono.ChSystemMulticore()
time_step = 0.01


app = irr.ChIrrApp(system, 'Gator Simulation', irr.dimension2du(800, 600))
app.AddTypicalLights()
app.SetCamera(chrono.ChVectorD(0, 5, 10))  
app.AssetSetMode(chrono.ChVisualizationAsset.Mode_RGB)
app.SetSymbolsScale(0.01)
app.SetTimestep(time_step)


gator = veh.Gator()
gator.SetChassisVisualizationType(veh.VisualizationType.MESH)
gator.SetWheelVisualizationType(veh.VisualizationType.SIMPLE)
gator.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
gator.SetPowertrain(veh.SimplePowertrain())  
gator.SetTireModel(veh.TireModel.SIMPLE)     


terrain = veh.RigidTerrain(system, veh.materials.Concrete())
patch = veh.Patch()
patch.m_length = 100
patch.m_width = 100
patch.m_friction = 0.9
terrain.AddPatch(patch)
terrain.Initialize()
system.Add(terrain)


gator.Initialize(terrain)


system.Add(gator.GetChassis())


driver = veh.Driver()
driver.SetChassis(gator.GetChassis())
gator.SetDriver(driver)


sensor_mgr = sens.SensorManager(system)
base_link = gator.GetChassis()
sensor_mgr.SetBaseLink(base_link)


light = sens.SensorLight()
light.SetName('point_light')
light.SetLightType(sens.SensorLightType.POINT)
light.SetPosition(chrono.ChVectorD(0, 0, 2))  
light.SetIntensity(1000)
sensor_mgr.AddSensor(light)


camera = sens.SensorCamera()
camera.SetName('front_camera')
camera.SetPosition(chrono.ChVectorD(0, 0, 1))  
camera.SetOrientation(chrono.Q_from_Euler_123(chrono.ChVectorD(0, 0, 0)))  
camera.SetFOV(90)
camera.SetImageWidth(800)
camera.SetImageHeight(600)
camera.SetSensorRenderingType(sens.SensorRenderType.RGB)
sensor_mgr.AddSensor(camera)

sensor_mgr.Initialize()


frame = 0
while app.GetDevice().run():
    
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

    
    gator.Update()
    sensor_mgr.Update()

    
    system.DoStepDynamics(time_step)

    
    app.BeginScene()
    app.DrawAll()
    terrain.Draw(app)
    app.EndScene()

    
    filename = f'output/frame_{frame:05d}.jpg'
    sensor_mgr.AddCameraImage(camera.GetName(), filename)
    frame += 1

    
    if app.GetDevice().getKey(irr.KEY_ESCAPE):
        break

app.GetDevice().closeGraphicalWindow()