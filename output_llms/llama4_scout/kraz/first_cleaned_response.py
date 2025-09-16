import chrono as ch
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


system = ch.ChSystem()


terrain = ch.ChBodyEasyTerrain(
    100, 100,  
    20,  
    ch.ChVector(0, 0, -9.81),  
    "terrain.jpg",  
    1,  
    0.7  
)
terrain.SetPos(ch.ChVector(0, 0, 0))
system.Add(terrain)


vehicle = ch.ChVehicle()
vehicle.Initialize(ch.ChCoordsys(ch.ChVector(0, 0, 1), ch.ChQuaternion(1, 0, 0, 0)))


system.Add(vehicle.GetChassisBody())
for wheel in vehicle.GetWheels():
    system.Add(wheel)
for axle in vehicle.GetAxles():
    system.Add(axle)


driver = ch.ChDriver()
driver.Initialize(vehicle, ch.ChVehicle.DriverInputs())


vis = irr.ChIrrApp(system, "Kraz Vehicle Simulation", 800, 600)
vis.SetCameraLinkType(irr.CameraLinkType.FOLLOW_BODY)
vis.SetCameraTarget(vehicle.GetChassisBody().GetPos())
vis.SetCameraPos(ch.ChVector(0, -10, 5))
vis.EnableCollisionVisualization(True)
vis.EnableRigidBodyVisualization(True)


vis.GetSceneManager().GetParameters().setAmbientLight(ch.ChColor(0.2, 0.2, 0.2))
vis.GetSceneManager().AddLight(irr.ChIrrLight(vehicle.GetChassisBody().GetPos(), ch.ChColor(1, 1, 1), 10))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(0.02)  
    driver.ComputeDriverInputs(0.02)
    vehicle.Update(0.02)
    vis.Synchronize(vehicle.GetChassisBody().GetPos())