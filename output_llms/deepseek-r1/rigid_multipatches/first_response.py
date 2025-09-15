import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
from pychrono import ChVector, ChCoordsys

# ===============================
# Initialize simulation environment
# ===============================

chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))
system = chrono.ChSystemNSC()
system.Set_G_acc(ChVector(0, -9.81, 0))

# ===============================
# Vehicle initialization
# ===============================

# Create HMMWV vehicle
vehicle = veh.HMMWV_Vehicle("HMMWV", veh.ChContactMethod_NSC)
vehicle.Initialize(ChCoordsys(ChVector(0, 0.5, 0), chrono.QUNIT))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Configure powertrain
engine = veh.SetEngineTypeSHAFTS(vehicle.GetVehicle(), veh.EngineModelType_SHAFTS)
transmission = veh.SetDrivelineTypeRWD(vehicle.GetVehicle(), veh.DrivelineTypeWV_SHAFTS)
vehicle.GetVehicle().InitializePowertrain(engine, transmission)

# ===============================
# Terrain construction
# ===============================

# Create rigid terrain manager
terrain = veh.RigidTerrain(system)

# Flat textured patch 1
patch1 = terrain.AddPatch(chrono.ChCoordsys(ChVector(-10, 0, 0), chrono.QUNIT), 
                        chrono.ChVector(20, 1, 10))
patch1.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 10)
patch1.SetFriction(0.8)

# Flat textured patch 2 (different friction/texture)
patch2 = terrain.AddPatch(chrono.ChCoordsys(ChVector(10, 0, 0), chrono.QUNIT), 
                        chrono.ChVector(20, 1, 10))
patch2.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 10)
patch2.SetFriction(0.6)

# Mesh-based bump patch
bump_mesh = chrono.ChTriangleMeshConnected()
bump_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/bump.obj"))
bump_body = chrono.ChBody()
bump_body.SetPos(ChVector(5, 0.2, 0))
bump_shape = chrono.ChTriangleMeshShape(bump_mesh)
bump_body.AddAsset(bump_shape)
bump_body.SetBodyFixed(True)
system.Add(bump_body)

# Heightmap terrain patch
heightmap = chrono.ChBodyEasyHeightMap(
    chrono.GetChronoDataFile("heightmaps/bump64.png"),
    64, 64,  # dimensions
    20, 10,   # x,y scaling
    0,        # vertical scaling
    True, True
)
heightmap.SetPos(ChVector(0, 0, -15))
system.Add(heightmap)

terrain.Initialize()

# ===============================
# Visualization setup
# ===============================

vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV Complex Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(ChVector(0, 3, -6))
vis.AddTypicalLights()

# ===============================
# Driver system setup
# ===============================

driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)
driver.Initialize()

# ===============================
# Simulation loop
# ===============================

time_step = 0.02
while vis.Run():
    time = system.GetChTime()
    
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    vehicle.GetVehicle().Synchronize(time, driver_inputs)
    
    # Advance simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    
    # Update visualization
    vehicle.GetVehicle().Advance(time_step)
    vis.Synchronize(time, driver_inputs)
    vis.Advance(time_step)