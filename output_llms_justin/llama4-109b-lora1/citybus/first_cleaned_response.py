import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChairsThreading(True)
chrono.ChSetFrameRate(50)


vehicle_file = "CityBus/CityBus_Vehicle.json"
vehicle = veh.ChPartedChronoVehicle()
vehicle.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.1), chrono.ChQuaterniond(1, 0, 0, 0)))


vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetCollisionSystemType(vehicle.ChCollisionSystemType_BULLET)


vehicle.SetTireModelType(vehicle.ChTireModelType_TMEASY)
vehicle.SetTireStepSize(0.02)


vehicle.SetVisualization(vehicle.ChVisualizationType_MESH)
vehicle.SetChassisVisualizationType(vehicle.ChChassisVisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(vehicle.ChSuspensionVisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(vehicle.ChSteeringVisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(vehicle.ChWheelVisualizationType_MESH)


terrain = veh.ChTerrain()
terrain.Initialize(vehicle.GetSystem())
terrain.SetCollisionSystemType(vehicle.ChCollisionSystemType_BULLET)
terrain.SetVisualization(vehicle.ChVisualizationType_MESH)


terrain.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))


driver = veh.ChInteractiveDriverIRR(vehicle)
driver.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachVehicle(vehicle)
vis.SetWindowTitle("CityBus Simulation")
vis.SetWindowSize(800, 600)
vis.SetCameraVertical(chrono.ChVector3d(0, 0, 1.75))
vis.SetCameraDistance(6.0)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)


vehicle.EnableRealtime(True)
vehicle.Initialize()
terrain.Initialize()
driver.Initialize()


while vis.Run() and not driver.IsFinished():
    time = vis.GetSimulationSteps()
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    driver.Synchronize(time)
    vehicle.Synchronize(time)
    terrain.Synchronize(time)
    driver.Advance(chrono.GetChTimeStep())
    vehicle.Advance(chrono.GetChTimeStep())
    terrain.Advance(chrono.GetChTimeStep())