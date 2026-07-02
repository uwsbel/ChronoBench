import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.sensor as sensor
from pychrono import chronoirr

# Initialize the PyChrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Create and initialize the Gator vehicle
gator = vehicle.Gator(system)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.2), chrono.QUNIT))
gator.Initialize()

# Set visualization types for vehicle parts
gator.GetChassis().SetVisualizationType(vehicle.VisualizationType_MESH)
for wheel in gator.GetWheelList():
    wheel.SetVisualizationType(vehicle.VisualizationType_POINTS)
for susp in gator.GetSuspensionList():
    susp.SetVisualizationType(vehicle.VisualizationType_PRIMITIVES)
for tierod in gator.GetTierodList():
    tierod.SetVisualizationType(vehicle.VisualizationType_LINES)

# Add rigid terrain
terrain = vehicle.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create interactive driver system
driver = vehicle.KeyboardDriver(gator)
driver.Initialize()

# Set up sensor manager with lights and camera
manager = sensor.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1, 1, 1), 500)
manager.scene.AddPointLight(chrono.ChVector3f(10, 0, 100), chrono.ChColor(1, 1, 1), 500)

camera = sensor.ChCameraSensor(
    gator.GetChassisBody(),
    30,  # update rate (Hz)
    chrono.ChFrame<>(chrono.ChVector3d(1.5, 0, 1.8), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))),
    1280, 720  # resolution
)
camera.SetName("Vehicle Camera")
camera.SetLag(False)
camera.SetCollectionWindow(1 / 30.0)
manager.AddSensor(camera)

# Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('PyChrono Gator Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -6, 3), chrono.ChVector3d(0, 0, 1.2))
vis.AddTypicalLights()

# Simulation loop parameters
time = 0
step = 1e-3
realtime_timer = chrono.ChRealtimeStep(0.01)

# Main simulation loop
while vis.Run() and time < 100:
    # Update driver inputs
    driver.Update()
    
    # Synchronize vehicle and terrain
    gator.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    
    # Update sensor manager
    manager.Update()
    
    # Advance simulation
    system.DoStepDynamics(step)
    time += step
    
    # Render visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Maintain real-time
    realtime_timer.Spin(step)