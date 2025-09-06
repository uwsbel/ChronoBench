import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def handle_keyboard(key, pressed, driver):
    if key == irr.KEY_KEY_W and pressed:
        driver.SetThrottle(1.0)
    elif key == irr.KEY_KEY_S and pressed:
        driver.SetThrottle(-1.0)
    elif key == irr.KEY_KEY_A and pressed:
        driver.SetSteering(-1.0)
    elif key == irr.KEY_KEY_D and pressed:
        driver.SetSteering(1.0)
    elif key == irr.KEY_KEY_SPACE and pressed:
        driver.SetBraking(1.0)
    else:
        driver.SetThrottle(0)
        driver.SetSteering(0)
        driver.SetBraking(0)

# Initialize Chrono system
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Setup visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1280, 1024)
vis.SetWindowTitle('Gator Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.SetCameraPosition(chrono.ChVectorD(0, 3, 10))
vis.AddTypicalLights()
vis.AddPointLight(chrono.ChVectorD(0, 0, 5), 2, 1, 1, 1)
vis.AddPointLight(chrono.ChVectorD(0, 0, -5), 2, 1, 1, 1)
vis.SetKeyboardCB(lambda key, pressed: handle_keyboard(key, pressed, driver))

# Create Gator vehicle
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod.NSC)
gator.SetChassisCollisionType(veh.ChassisCollisionType.BOX)
gator.SetWheelCollisionType(veh.WheelCollisionType.BOX)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
gator.SetVisualizationType(veh.VisualizationType_MESH)
gator.Initialize()

# Setup terrain
material = veh.CreateMaterial(0.9, 2e7, 0.1)
terrain = veh.RigidTerrain(my_system)
patch = terrain.AddPatch(material, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100)
terrain.Initialize()

# Initialize driver
driver = veh.ManualDriver()
gator.SetDriver(driver)

# Setup sensor manager with camera
sensor_manager = sens.ChSensorManager(my_system)
sensor_manager.SetRecordingFrequency(30)

chassis = gator.GetChassis()
camera = sens.ChCameraSensor(sensor_manager, chassis)
camera.SetName("chassis_camera")
camera.SetPosition(chrono.ChVectorD(0, 0.5, 1))
camera.SetRotation(chrono.Q_from_Euler_123(chrono.ChVectorD(0, 0, 0)))
camera.SetFOV(90)
camera.SetImageWidth(800)
camera.SetImageHeight(600)
sensor_manager.AddSensor(camera)

# Simulation parameters
time_step = 0.001
my_system.SetTimestep(time_step)
time = 0

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update driver inputs
    gator.Synchronize(time, driver)
    gator.Update(time)

    # Update terrain
    terrain.Synchronize(time)

    # Update sensor manager
    sensor_manager.Update(time)

    # Advance simulation
    my_system.DoStepDynamics(time_step)
    time += time_step

vis.Close()