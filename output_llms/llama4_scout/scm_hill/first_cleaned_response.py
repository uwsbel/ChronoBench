import chrono as ch
import chrono.vehicle as veh
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


system = ch.ChSystem_EulerImplicit()


vehicle = veh.ChHMMWV(system)


vehicle.SetPos(ch.ChVector3d(0, 0, 1.0))
vehicle.SetRot(ch.ChQuaterniond(1, 0, 0, 0))


terrain = veh.ChSCMTerrain(system)


soil = terrain.GetSoilModel()
soil.SetFriction(0.8)
soil.SetRestitution(0.2)
soil.SetDamping(0.5)


height_map = []
for x in range(-100, 101):
    row = []
    for y in range(-100, 101):
        row.append(0.5)  
    height_map.append(row)
terrain.Initialize(height_map)


driver = veh.ChDriver(system, vehicle.GetChassisBody())
driver.SetThrottle(0.5)
driver.SetSteering(0.2)


vis = irr.ChIrrApp(system, 'HMMWV SCM Terrain Simulation', 800, 600)


vis.SetCameraPosition(ch.ChVector3d(0, -5, 1))
vis.SetCameraTarget(ch.ChVector3d(0, 0, 0))
vis.EnableSun()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(0.01)  
    driver.Update(0.01)
    vis.Synchronize(0.01)

vis.Exit()