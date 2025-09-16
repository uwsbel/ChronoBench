import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np

# Initialize the PyChrono environment
chrono.SetChronoDataPath('./data/')

# Create the simulation system and add it to the ChSystem
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the vehicle and add it to the system
vehicle = veh.ChCityBus()
vehicle.SetChassisFixed(False)
vehicle.SetChassisMass(1500)
vehicle.SetChassisInertiaXX(chrono.ChVectorD(100, 100, 100))
vehicle.SetChassisInertiaXY(chrono.ChVectorD(0, 0, 0))
vehicle.SetChassisInertiaXZ(chrono.ChVectorD(0, 0, 0))
vehicle.SetChassisInertiaYY(chrono.ChVectorD(100, 100, 100))
vehicle.SetChassisInertiaYZ(chrono.ChVectorD(0, 0, 0))
vehicle.SetChassisInertiaZZ(chrono.ChVectorD(100, 100, 100))
vehicle.SetChassisPosition(chrono.ChVectorD(0, 0, 0.5))
vehicle.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.SetTireModel(veh.TireModelType.TM_RIGID)

# Create the rigid terrain and add it to the system
terrain = veh.ChRigidTerrain(system)
terrain.SetTexture(chrono.GetChronoDataFile('terrain/textures/tile_0.jpg'))
terrain.SetPlane(chrono.ChPlane(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0)))

# Create the visualization system and add it to the ChSystem
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.AddSkyBox()
vis.AddTypicalLights()

# Create the vehicle visualization
vehicle_vis = veh.ChCityBusVisualization(vehicle)
vehicle_vis.SetChassisVisualizationType(chronoirrVisualizationType.MESH)
vehicle_vis.SetWheelVisualizationType(chronoirrVisualizationType.PRIMITIVE)
vehicle_vis.SetSteeringVisualizationType(chronoirrVisualizationType.PRIMITIVE)
vehicle_vis.SetSuspensionVisualizationType(chronoirrVisualizationType.PRIMITIVE)
vehicle_vis.SetChassisColor(chrono.ChColor(1, 0, 0))

# Create the terrain visualization
terrain_vis = veh.ChRigidTerrainVisualization(terrain)
terrain_vis.SetVisualizationType(chronoirrVisualizationType.MESH)
terrain_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))

# Create the camera and set it to follow the vehicle
camera = vis.AddCamera(chrono.ChVectorD(0, 0, 2))
camera.SetFollowNode(vehicle.GetChassisBody())
camera.SetLookAtNode(vehicle.GetChassisBody())

# Create the interactive driver system
driver = veh.ChIrrlichtDriver()
driver.SetVehicle(vehicle)
driver.SetSteeringGain(0.5)
driver.SetThrottleGain(0.5)
driver.SetBrakingGain(0.5)

# Simulation loop
while vis.Run():
    # Update the vehicle dynamics
    vehicle.Update(1 / 50.0)
    terrain.Synchronize(1 / 50.0)
    
    # Render the scene
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    # Update the driver system
    driver.Synchronize(1 / 50.0)
    
    # Update the visualization system
    vis.DoStep(1 / 50.0)