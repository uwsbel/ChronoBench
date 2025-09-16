import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np


chrono.SetChronoDataPath('./data/')


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


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


terrain = veh.ChRigidTerrain(system)
terrain.SetTexture(chrono.GetChronoDataFile('terrain/textures/tile_0.jpg'))
terrain.SetPlane(chrono.ChPlane(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0)))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.AddSkyBox()
vis.AddTypicalLights()


vehicle_vis = veh.ChCityBusVisualization(vehicle)
vehicle_vis.SetChassisVisualizationType(chronoirrVisualizationType.MESH)
vehicle_vis.SetWheelVisualizationType(chronoirrVisualizationType.PRIMITIVE)
vehicle_vis.SetSteeringVisualizationType(chronoirrVisualizationType.PRIMITIVE)
vehicle_vis.SetSuspensionVisualizationType(chronoirrVisualizationType.PRIMITIVE)
vehicle_vis.SetChassisColor(chrono.ChColor(1, 0, 0))


terrain_vis = veh.ChRigidTerrainVisualization(terrain)
terrain_vis.SetVisualizationType(chronoirrVisualizationType.MESH)
terrain_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))


camera = vis.AddCamera(chrono.ChVectorD(0, 0, 2))
camera.SetFollowNode(vehicle.GetChassisBody())
camera.SetLookAtNode(vehicle.GetChassisBody())


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(vehicle)
driver.SetSteeringGain(0.5)
driver.SetThrottleGain(0.5)
driver.SetBrakingGain(0.5)


while vis.Run():
    
    vehicle.Update(1 / 50.0)
    terrain.Synchronize(1 / 50.0)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    driver.Synchronize(1 / 50.0)
    
    
    vis.DoStep(1 / 50.0)