import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens


chrono.SetChronoDataPath(chrono.GetChronoDataPath() + "/../data/")
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


vehicle = veh.Gator()
vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1.5))
vehicle.SetInitOrientation(chrono.ChQuaternionD(1, 0, 0, 0))


vehicle.SetTireModelType(veh.TireModelType.TMEASY)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.SetWindowSize(1024, 768)
vis.Initialize()
vis.AddLogo()
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)


terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain_length = 200.0    
terrain_width = 200.0     
terrain.SetPlaneDimensions(chrono.ChVector2D(terrain_length, terrain_width))


texture = chrono.GetChronoDataFile("sensor/textures/concrete.jpg")
terrain.SetTexture(texture)


terr_vis = veh.RigidTerrainVisualizationMesh(terrain)
terr_vis.SetMeshFile(chrono.GetChronoDataFile("sensor/meshes/terrain.obj"))
terr_vis.SetTexture(texture)
terrain.SetVisualizationMesh(terr_vis)


vehicle.GetSystem().Add(terrain)


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
driver.Initialize()


application = veh.ChRealtimeStepDynamics(vehicle.GetSystem())
application.SetTimestep(1 / 50)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    application.DoStep()
    driver.Synchronize(vehicle.GetVehicle().GetChTime())
    vis.EndScene()

vis.GetDevice().closeDevice()