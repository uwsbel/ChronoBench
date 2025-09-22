import chrono as ch
import chrono.irrlicht as irr
import chrono.vehicle as veh


ch.set_default_data_file_path("./chrono_assets")
ch.set_num_threads(1)


system = ch.ChSystem_EulerImplicit()


vehicle = veh.ChHMMWV(system)


vehicle.SetInitialPositionAndOrientation(ch.ChVector3d(0, 0, 1.1), ch.ChQuaterniond(1, 0, 0, 0))


terrain = veh.ChTerrain(system)
terrain.SetType(veh.TerrainType.SC_M)


soil = terrain.GetSoil()
soil.SetFriction(0.9)
soil.SetRestitution(0.3)
soil.SetDamping(0.5)


heightMap = []
for iz in range(20):
    row = []
    for ix in range(20):
        height = 0.1 * (ix + iz)
        row.append(height)
    heightMap.append(row)
terrain.Initialize(heightMap, 1.0, 20, 20)


driver = veh.ChDriver(vehicle)
driver.SetSteeringType(veh.DriverSteeringType_PID)
driver.SetSpeedType(veh.DriverSpeedType_PID)


driver.SetTargetSpeed(10)
driver.SetTargetSteering(0)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(irr.CameraVerticalDir.Z)
vis.SetCameraPosition(ch.ChVector3d(0, -5, 1))
vis.SetLightDirection(ch.ChVector3d(1, -1, 1))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver.Synchronize(vis.GetChTimeStep())
    vehicle.Synchronize(vis.GetChTimeStep())
    terrain.Synchronize(vis.GetChTimeStep())
    vis.Synchronize(vis.GetChTimeStep())

    system.DoStepDynamics(vis.GetChTimeStep())

    
    vis.UpdateTimeStep()