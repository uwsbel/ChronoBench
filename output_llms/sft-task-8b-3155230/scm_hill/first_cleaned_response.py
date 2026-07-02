import pychrono as chrono
from pychrono.vehicle import ChSystemSMC, ChHMMWV, ChDriver, ChVisualSystemIrrlicht
import numpy as np


system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


vehicle = ChHMMWV()
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChVector3d(0, 0.5, 1))
vehicle.Initialize(system)
vehicle.SetTireType(ChHMMWV.TireType.RIGID)
vehicle.SetChassisVisualizationType(ChHMMWV.VisualizationType.NONE)
vehicle.SetSuspensionVisualizationType(ChHMMWV.VisualizationType.NONE)
vehicle.SetSteeringVisualizationType(ChHMMWV.VisualizationType.NONE)
vehicle.SetWheelVisualizationType(ChHMMWV.VisualizationType.NONE)


terrain = chrono.ChBody()
terrain.SetName("SCM Terrain")
terrain.SetBodyFixed(False)
terrain.EnableCollision(True)
terrain.SetMass(0)
terrain.UseMaterialSurface(True)
terrain.GetMaterialSurface().SetFriction(0.8)
terrain.GetMaterialSurface().SetRestitution(0.2)
terrain.GetMaterialSurface().SetDamping(0.1)
terrain.GetMaterialSurface().SetDensity(1000)
terrain.GetMaterialSurface().SetStiffness(1e5)
terrain.GetMaterialSurface().SetDamping(1e3)
terrain.GetMaterialSurface().SetThickness(0.1)
terrain.GetMaterialSurface().SetSoilType(chrono.ChMaterialSurface.SCM)
terrain.SetPos(chrono.ChVector3d(0, 0, 0))
system.Add(terrain)


height_map = np.zeros((100, 100))
for i in range(100):
    for j in range(100):
        height_map[i, j] = 0.1 * np.sin(0.1 * i) * np.cos(0.1 * j)
terrain.SetHeightMap(height_map, 10.0, 10.0)


driver = ChDriver(vehicle.GetVehicle())
driver.Initialize()
driver.SetThrottle(0.1)
driver.SetSteering(0.0)


visual_system = ChVisualSystemIrrlicht()
visual_system.AttachSystem(system)
visual_system.SetWindowSize(1024, 768)
visual_system.SetWindowTitle("HMMWV on SCM Terrain")
visual_system.Initialize()
visual_system.AddSkyBox()
visual_system.AddCamera(chrono.ChVector3d(10, 5, 2), chrono.ChVector3d(0, 0, 0))
visual_system.AddLightDirectional(chrono.ChVector3d(0, 1, 0), chrono.ChColor(0.8, 0.8, 0.8))


vehicle.AddVisualizationAssets(ChHMMWV.VisualizationType.BASIC)
visual_system.AddAsset(vehicle.GetVisualizationAsset())


time_step = 1e-3
while visual_system.Run():
    time = system.GetChTime()
    driver.Synchronize(time)
    driver.Advance(time_step)
    vehicle.Synchronize(time, driver.GetInputs())
    vehicle.Advance(time_step)
    terrain.Advance(time_step)
    visual_system.BeginScene()
    visual_system.Render()
    visual_system.EndScene()
    system.DoStepDynamics(time_step)