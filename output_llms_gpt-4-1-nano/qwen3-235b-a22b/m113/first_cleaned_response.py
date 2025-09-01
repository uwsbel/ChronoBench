import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


vehicle = veh.M113_Vehicle(system)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.SetPowertrainType(veh.PowertrainModelType_SIMPLE_MAP)
vehicle.SetDriveType(veh.DrivelineTypeTV_FULL)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSprocketVisualizationType(veh.VisualizationType_MESH)
vehicle.SetIdlerVisualizationType(veh.VisualizationType_MESH)
vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(vehicle.GetSystem())
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.9)
material.SetRestitution(0.05)
patch = terrain.AddPatch(material, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 200, 100)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()


driver = veh.M113_Driver(vehicle)
driver.Initialize()


application = irr.ChIrrApp(vehicle.GetSystem(), 'M113 Vehicle Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddTypicalSky()


camera = irr.ChIrrCamera(application.GetDevice(), vehicle.GetChassisBody(), 10.0, 0.5)
application.AddCamera(camera)


step_size = 0.01
timer = chrono.ChRealtimeStepTimer()


application.SetStepManage(True)
application.SetTimestep(step_size)
application.SetTryRealtime(True)

while application.GetDevice().run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    driver.Synchronize(time)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    system.DoStepDynamics(step_size)
    timer.Spin(step_size)