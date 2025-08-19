import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Initialize PyChrono environment and core components
chrono.SetChairsThreading(True)
chrono.ChSetFrameRate(50)

# Create the vehicle, set parameters, and initialize
vehicle_file = "CityBus/CityBus_Vehicle.json"
vehicle = veh.ChPartedChronoVehicle()
vehicle.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.1), chrono.ChQuaterniond(1, 0, 0, 0)))

# Set the contact method
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetCollisionSystemType(vehicle.ChCollisionSystemType_BULLET)

# Set the tire model
vehicle.SetTireModelType(vehicle.ChTireModelType_TMEASY)
vehicle.SetTireStepSize(0.02)

# Set the vehicle visualization type
vehicle.SetVisualization(vehicle.ChVisualizationType_MESH)
vehicle.SetChassisVisualizationType(vehicle.ChChassisVisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(vehicle.ChSuspensionVisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(vehicle.ChSteeringVisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(vehicle.ChWheelVisualizationType_MESH)

# Create the terrain
terrain = veh.ChTerrain()
terrain.Initialize(vehicle.GetSystem())
terrain.SetCollisionSystemType(vehicle.ChCollisionSystemType_BULLET)
terrain.SetVisualization(vehicle.ChVisualizationType_MESH)

# Set the terrain texture
terrain.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vehicle)
driver.Initialize()

# Create the Irrlicht application
vis = irr.ChVisualSystemIrrlicht()
vis.AttachVehicle(vehicle)
vis.SetWindowTitle("CityBus Simulation")
vis.SetWindowSize(800, 600)
vis.SetCameraVertical(chrono.ChVector3d(0, 0, 1.75))
vis.SetCameraDistance(6.0)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)

# Initialize the simulation
vehicle.EnableRealtime(True)
vehicle.Initialize()
terrain.Initialize()
driver.Initialize()

# Simulation loop
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