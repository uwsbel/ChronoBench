import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

# Initialize the PyChrono system
my_system = chrono.ChSystemMulticore()

# Create and configure the Gator vehicle
gator = veh.Gator(vehicle_system=my_system)
gator.SetChassisVisualizationType(veh.VisualizationType.MESH)
gator.SetWheelVisualizationType(veh.VisualizationType.MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType.PRIMITIVES)
gator.SetPowertrainVisualizationType(veh.VisualizationType.NONE)
gator.Initialize()

# Create a rigid terrain (plane)
ground = chrono.ChBodyEasyPlane(100, 100, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -1))
my_system.Add(ground)

# Initialize the Irrlicht visualization
irr_params = irr.ChIrrAppParams()
irr_params.SetWindowSize(1280, 1024)
irr_params.SetWindowTitle("Gator Simulation")
irr_params.SetLightIntensity(0.8)
irr_params.SetCameraPosition(chrono.ChVectorD(0, 5, 10))

app = irr.ChIrrApp(my_system, irr_params)
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera()

# Add vehicle and terrain to the scene
app.Add(gator.GetChassis())
app.Add(ground)

# Configure interactive driver
driver = irr.ChIrrGuiDriver()
driver.SetVehicle(gator)
driver.SetChaseCamera(gator.GetChassis(), 6.0, 0.5)
app.SetDriver(driver)

# Initialize sensor manager with camera and lights
sensor_mgr = sens.ChSensorManager(my_system)
sensor_mgr.SetSceneType(sens.SENSOR_SCANTYPE_RGBD)

# Camera sensor attached to the chassis
cam = sens.ChCameraSensor()
cam.SetName("chassis_camera")
cam.SetFocalDistance(100)
cam.SetFov(90)
cam.SetImageWidth(640)
cam.SetImageHeight(480)
cam.SetPosition(chrono.ChVectorD(0, 0, 1))  # Position relative to chassis
cam.SetRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X))  # Rotate to look forward
cam.SetAttachmentFrame(gator.GetChassis().GetFrame_REF_to_abs())
sensor_mgr.AddSensor(cam)

# Add point lights for illumination
light1 = sens.ChLight()
light1.SetPosition(chrono.ChVectorD(5, 5, 10))
light1.SetIntensity(200)
sensor_mgr.AddLight(light1)

light2 = sens.ChLight()
light2.SetPosition(chrono.ChVectorD(-5, -5, 10))
light2.SetIntensity(200)
sensor_mgr.AddLight(light2)

# Simulation loop
app.Begin()
while app.GetDevice().run():
    time = my_system.GetChTime()
    
    # Update driver inputs and synchronize with vehicle
    driver.Synchronize(time)
    driver.Update()
    
    # Update vehicle state
    gator.Synchronize(time, driver)
    gator.Update(time)
    
    # Update sensor manager (process camera)
    sensor_mgr.Update()
    
    # Advance simulation step
    my_system.DoStepDynamics(0.005)
    
    # Render scene
    app.Render()
    
    # Retrieve and process camera image (example: print dimensions)
    image = cam.GetLastRenderedImage()
    if image:
        print(f"Camera image dimensions: {image.GetWidth()}x{image.GetHeight()}")
    
    # Optional exit condition after 10 seconds
    if time > 10:
        break

app.End()