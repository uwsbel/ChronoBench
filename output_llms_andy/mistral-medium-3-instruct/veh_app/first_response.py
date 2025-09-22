import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# Set the path to the Chrono data directory
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Update this path

# Create the simulation system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create the terrain
terrain = veh.RigidTerrain(system)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain.Initialize(chrono.GetChronoDataFile("terrain/flat_terrain.obj"), chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))

# Create the Gator vehicle
gator = veh.Gator(system)
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordys(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
gator.Initialize()

# Set visualization types for various parts
gator.GetChassisBody().SetVisualizationType(chrono.VisualizationType_MESH)
gator.GetSteeringLink(0).SetVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.GetSuspension(0).GetSpindleBody().SetVisualizationType(chrono.VisualizationType_WIREFRAME)

# Create the interactive driver system
driver = veh.ChInteractiveDriverIRR()
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# Create the sensor manager
sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetRecording(True)

# Add a camera sensor to the vehicle chassis
camera = sens.ChCameraSensor(
    gator.GetChassisBody(),
    10,  # update rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngX(chrono.CH_C_PI_2)),
    640,  # width
    480,  # height
    chrono.CH_C_PI / 4,  # fov
    100.0  # range
)
sensor_manager.AddSensor(camera)

# Add point lights
light1 = sens.ChPointLight(
    gator.GetChassisBody(),
    chrono.ChVectorD(0.5, 0.5, 1.5),
    chrono.ChColor(1, 1, 1),
    10.0
)
sensor_manager.AddSensor(light1)

light2 = sens.ChPointLight(
    gator.GetChassisBody(),
    chrono.ChVectorD(-0.5, -0.5, 1.5),
    chrono.ChColor(1, 1, 1),
    10.0
)
sensor_manager.AddSensor(light2)

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gator Vehicle Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 2), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()
vis.SetCameraMovementMode(chronoirr.CameraMovementMode_FREE)

# Create the application
app = chronoirr.ChIrrApp(system, vis, "Gator Vehicle Simulation")
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(5, 5, 2), chrono.ChVectorD(0, 0, 0.5))
app.AssetBindAll()
app.AssetUpdateAll()

# Simulation loop
time_step = 0.01
max_time = 10.0
current_time = 0.0

while app.GetDevice().run() and current_time < max_time:
    # Update time
    current_time += time_step

    # Update driver inputs
    driver.Synchronize(current_time)
    driver_input = driver.GetInputs()

    # Update vehicle
    gator.Synchronize(current_time, driver_input.steering, driver_input.throttle, driver_input.braking)
    gator.Update(current_time)

    # Update sensor manager
    sensor_manager.Update()

    # Advance simulation
    app.BeginScene()
    app.DrawAll()
    app.DoStepDynamics(time_step)
    app.EndScene()

    # Print camera image (simplified - in practice you would process the image data)
    if camera.GetRecord():
        print("Camera image captured at time:", current_time)

# Clean up
sensor_manager.WriteSensorData()