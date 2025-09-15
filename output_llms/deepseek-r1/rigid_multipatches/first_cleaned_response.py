import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))


def create_terrain(system):
    
    concrete_box = chrono.ChBodyEasyBox(20, 20, 0.2, 2000, True, True)
    concrete_box.SetPos(chrono.ChVectorD(-10, 0, -0.1))
    concrete_box.SetBodyFixed(True)
    concrete_box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(concrete_box)

    
    grass_box = chrono.ChBodyEasyBox(20, 20, 0.2, 2000, True, True)
    grass_box.SetPos(chrono.ChVectorD(20, 0, -0.1))
    grass_box.SetBodyFixed(True)
    grass_box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"))
    system.Add(grass_box)

    
    bump_mesh = chrono.ChTriangleMeshConnected()
    bump_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/speed_bump.obj"))
    bump_body = chrono.ChBody()
    bump_body.SetPos(chrono.ChVectorD(5, 0, 0))
    bump_body.SetBodyFixed(True)
    bump_shape = chrono.ChTriangleMeshShape()
    bump_shape.SetMesh(bump_mesh)
    bump_shape.SetTexture(chrono.GetChronoDataFile("textures/stone.jpg"))
    bump_body.AddVisualShape(bump_shape)
    bump_body.GetCollisionModel().ClearModel()
    bump_body.GetCollisionModel().AddTriangleMesh(bump_mesh, False, False)
    bump_body.GetCollisionModel().BuildModel()
    bump_body.SetCollide(True)
    system.Add(bump_body)

    
    heightmap = chrono.ChBodyEasyHeightmap(
        chrono.GetChronoDataFile("heightmaps/bump.jpg"),  
        20, 20,          
        0, 1.0,          
        True, True       
    )
    heightmap.SetPos(chrono.ChVectorD(5, 15, -0.5))
    heightmap.SetBodyFixed(True)
    heightmap.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/dirt.jpg"))
    system.Add(heightmap)


def create_vehicle(system):
    
    init_pos = chrono.ChVectorD(0, 0, 0.5)
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    
    
    vehicle.Initialize()
    
    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    system.Add(vehicle.GetVehicle())


def setup_visualization(system):
    app = veh.ChWheeledVehicleIrrApp(system.GetVehicle(), "HMMWV Simulation")
    app.AddTypicalLights()
    app.SetChaseCamera(chrono.ChVectorD(0, 0, 1.5), 6, 0.5)
    app.SetTimestep(0.02)
    app.AssetBindAll()
    app.AssetUpdateAll()
    return app


def create_driver(app):
    driver = veh.ChIrrGuiDriver(app)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    return driver


create_terrain(system)
create_vehicle(system)
app = setup_visualization(system)
driver = create_driver(app)


while app.GetDevice().run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    system.GetVehicle().GetDriver().SetInputs(driver_inputs)
    
    
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.Advance(time)
    app.EndScene()
    system.DoStepDynamics(0.01)