import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))


vehicle = veh.MAN_Kat10t(system)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType_TMeasy)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.Initialize()


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(200, 200, 1))
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile.png"), 200, 200)
terrain.Initialize()


driver = veh.ChDriver(vehicle.GetVehicle())


application = irr.ChIrrApp(system, 'MAN 10t Simulation', irr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chrono.ChVector3d(0, -5, 3), chrono.ChVector3d(0, 0, 1))
application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    
    throttle = 0.0
    braking = 0.0
    steering = 0.0
    
    if application.GetDevice().isKeyDown(irr.KEY_UP):
        throttle = 0.7
    if application.GetDevice().isKeyDown(irr.KEY_DOWN):
        braking = 0.7
    if application.GetDevice().isKeyDown(irr.KEY_LEFT):
        steering = -0.7
    elif application.GetDevice().isKeyDown(irr.KEY_RIGHT):
        steering = 0.7
        
    driver.SetThrottle(throttle)
    driver.SetBraking(braking)
    driver.SetSteering(steering)
    
    
    time = chrono.ChTime.GetTime()
    driver.Synchronize(time)
    vehicle.Synchronize(time)
    terrain.Synchronize(time)
    
    
    system.DoStepDynamics(0.01)
    
    driver.Advance(0.01)
    vehicle.Advance(0.01)
    terrain.Advance(0.01)
    
    
    application.BeginScene()
    application.DrawAll()
    
    
    vehicle_pos = vehicle.GetVehicle().GetPos()
    camera_pos = vehicle_pos + chrono.ChVector3d(0, -5, 3)
    irr_camera = application.GetSceneManager().getActiveCamera()
    irr_camera.setPosition(irr.vector3df(camera_pos.x, camera_pos.y, camera_pos.z))
    irr_camera.setTarget(irr.vector3df(vehicle_pos.x, vehicle_pos.y, vehicle_pos.z))
    
    application.EndScene()