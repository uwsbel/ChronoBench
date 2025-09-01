import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# Set data path to Chrono data directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# Simulation parameters
step_size = 0.02
render_step_size = 0.02

# Create Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetSolverMaxIterations(100)

# Initialize HMMWV vehicle
hmmwv = veh.HMMWV_Vehicle()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.QUNIT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_RWD)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.Initialize()

# Set visualization type to MESH for all components
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# Create complex terrain
terrain = veh.RigidTerrain(system)

# Flat concrete patch
patch1 = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(20, 10)
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 10)
patch1.SetFriction(0.9)

# Flat dirt patch
patch2 = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(20, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(20, 10)
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 10)
patch2.SetFriction(0.7)

# Mesh-based bump (triangular obstacle)
mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVectorD(40, 0, 0.2))
mesh_body.SetBodyFixed(True)
mesh = chrono.ChTriangleMeshConnected()
mesh.AddVertex(chrono.ChVectorD(-1, -0.5, 0))
mesh.AddVertex(chrono.ChVectorD(1, -0.5, 0))
mesh.AddVertex(chrono.ChVectorD(0, -0.5, 0.5))
mesh.AddTriangle(0, 1, 2)
mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
mesh_body.AddAsset(mesh_shape)
mesh_body.GetCollisionModel().ClearModel()
mesh_body.GetCollisionModel().AddTriangleMesh(mesh, True, False)
mesh_body.GetCollisionModel().BuildModel()
mesh_body.SetCollide(True)
system.Add(mesh_body)

# Heightmap terrain patch
patch3 = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(42, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/heightmaps/bump64.bmp"),
    64, 64, 0, 2
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"))
patch3.SetFriction(0.8)

terrain.Initialize()

# Create Irrlicht visualization
app = irr.ChIrrApp(system, "HMMWV Simulation", irr.dimension2du(1280, 720))
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0, 3, 6), irr.vector3df(0, 0, 0))
app.AssetBindAll()
app.AssetUpdateAll()

# Create interactive driver
driver = veh.ChIrrGuiDriver(app)
steering_time = 1.0  # Time to go from 0 to +1 (or -1)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(0.5 * render_step_size)
driver.SetBrakingDelta(0.5 * render_step_size)
driver.Initialize()

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
while app.GetDevice().run():
    time = system.GetChTime()
    
    # Render scene
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update vehicle systems
    hmmwv.GetVehicle().Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    
    # Update visualization
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs)
    
    # Maintain real-time simulation
    realtime_timer.Spin(step_size)

print("Simulation completed.")