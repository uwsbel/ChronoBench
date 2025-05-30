import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as irr

# Initialize the PyChrono system
my_system = chrono.ChSystemNSC()

# Set up Irrlicht visualization
app = irr.ChIrrApp(my_system, 'Gator Vehicle Simulation', irr.dimension2du(1280, 720))
app.AddTypicalLogo()
app.AddTypicalCamera(irr.vector3df(0, 3, -5), irr.vector3df(0, 0, 0))
app.AddTypicalLights()

# Create the Gator vehicle
gator = veh.Gator(my_system)
gator.SetInitPosition(chrono.ChCoordsys(chrono.ChVector(0, 0.5, 0), chrono.QUNIT))
gator.SetEngineType(veh.EngineModelType_SIMPLE)
gator.SetTransmissionType(veh.TransmissionModelType_SIMPLE_MAP)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(0.001)
gator.Initialize()

# Configure visualization types for vehicle components
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# Create rigid terrain
terrain = veh.RigidTerrain(my_system)
patch = terrain.AddPatch(chrono.ChCoordsys(), chrono.ChVector(200, 1, 200), 0.8)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()

# Set up interactive driver
driver = veh.ChIrrGuiDriver(app.GetDevice(), gator)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)

# Configure sensor manager with camera and lights
manager = sens.ChSensorManager(my_system)

# Add a camera sensor attached to the chassis
camera = sens.ChCameraSensor(
    gator.GetChassisBody(),
    60,  # Update rate (Hz)
    chrono.ChFrame(chrono.ChVector(0, 1.5, 2), chrono.Q_from_AngAxis(0, chrono.ChVector(0, 1, 0))),
    1280, 720,  # Resolution
    chrono.CH_C_PI / 3  # Field of view
)
camera.SetName("Front Camera")
manager.AddSensor(camera)

# Add a point light source
light = sens.ChPointLight(chrono.ChVector(2, 3, 2), chrono.ChColor(1, 1, 1))
manager.AddLight(light)

# Simulation loop
step_size = 0.001  # Integration step size
time = 0

while app.GetDevice().run():
    time = my_system.GetChTime()
    
    # Synchronize modules
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    manager.Update(time)
    
    # Advance simulation
    my_system.DoStepDynamics(step_size)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    manager.Advance(step_size)
    
    # Render the scene
    app.BeginScene()
    app.DrawAll()
    app.EndScene()