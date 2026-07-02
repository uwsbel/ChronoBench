import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.sensor as sensor
from pychrono import chronoirr


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


gator = vehicle.Gator(system)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.2), chrono.QUNIT))
gator.Initialize()


gator.GetChassis().SetVisualizationType(vehicle.VisualizationType_MESH)
for wheel in gator.GetWheelList():
    wheel.SetVisualizationType(vehicle.VisualizationType_POINTS)
for susp in gator.GetSuspensionList():
    susp.SetVisualizationType(vehicle.VisualizationType_PRIMITIVES)
for tierod in gator.GetTierodList():
    tierod.SetVisualizationType(vehicle.VisualizationType_LINES)


terrain = vehicle.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


driver = vehicle.KeyboardDriver(gator)
driver.Initialize()


manager = sensor.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1, 1, 1), 500)
manager.scene.AddPointLight(chrono.ChVector3f(10, 0, 100), chrono.ChColor(1, 1, 1), 500)

camera = sensor.ChCameraSensor(
    gator.GetChassisBody(),
    30,  
    chrono.ChFrame<>(chrono.ChVector3d(1.5, 0, 1.8), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))),
    1280, 720  
)
camera.SetName("Vehicle Camera")
camera.SetLag(False)
camera.SetCollectionWindow(1 / 30.0)
manager.AddSensor(camera)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('PyChrono Gator Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -6, 3), chrono.ChVector3d(0, 0, 1.2))
vis.AddTypicalLights()


time = 0
step = 1e-3
realtime_timer = chrono.ChRealtimeStep(0.01)


while vis.Run() and time < 100:
    
    driver.Update()
    
    
    gator.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    
    
    manager.Update()
    
    
    system.DoStepDynamics(step)
    time += step
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    realtime_timer.Spin(step)