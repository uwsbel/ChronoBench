import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import time

# Initialize the system with NSC solver
my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetMaxItersSolverSpeed(10)

# Visualization setup
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV on Mesh Terrain")
vis.SetCamera(chrono.ChVectorD(0, 5, 20))
vis.AddTypicalLights()
vis.Initialize()

# Create HMMWV with visualization and collision enabled
hmmwv = veh.HMMWV(True, True)  # Enable visualization and collision
hmmwv.SetContactMethod(chrono.ChContactMethod.NSC)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType.MESH)
hmmwv.SetTireType(veh.TireModelType.TMEASY)

# Initialize vehicle position and orientation
init_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT)
hmmwv.SetInitPosition(init_pos)
hmmwv.Initialize()

# Create terrain with custom mesh
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.9)
terrain = veh.RigidTerrain(my_system)
patch = terrain.AddPatch(material, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100, False, veh.VisualMaterial.GRASS)
terrain.Initialize()

# Modify terrain's ground body to use custom meshes
ground_body = terrain.GetGround()

# Load collision mesh (Highway_col.obj)
collision_mesh = chrono.ChTriangleMeshShape()
collision_mesh.SetMesh(chrono.ChTriangleMesh())
collision_mesh.GetMesh().LoadWavefrontMesh(chrono.GetChronoDataFile("Highway_col.obj"))
collision_mesh.Build()

# Load visual mesh (Highway_vis.obj)
visual_mesh = chrono.ChTriangleMeshShape()
visual_mesh.SetMesh(chrono.ChTriangleMesh())
visual_mesh.GetMesh().LoadWavefrontMesh(chrono.GetChronoDataFile("Highway_vis.obj"))
visual_mesh.Build()

# Replace ground's collision model
ground_body.GetCollisionModel().ClearModel()
ground_body.GetCollisionModel().AddTriangleMesh(collision_mesh.GetMesh())
ground_body.GetCollisionModel().BuildModel()
ground_body.GetCollisionModel().SetCollide(True)

# Replace ground's visual shapes
for i in range(ground_body.GetVisualShapeCount()):
    ground_body.GetVisualShape(i).Remove()
ground_body.AddVisualShape(visual_mesh)

# Interactive driver setup
driver = veh.KeyboardDriver()
hmmwv.SetDriver(driver)

# Simulation parameters
time_step = 0.001  # Simulation time step
frame_rate = 50
frame_time = 1.0 / frame_rate

# Simulation loop
start_time = time.time()
while vis.Run():
    # Update driver inputs
    driver.Synchronize(time_step)
    
    # Advance simulation
    my_system.DoStepDynamics(time_step)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Cap frame rate
    current_time = time.time()
    elapsed = current_time - start_time
    if elapsed < frame_time:
        time.sleep(frame_time - elapsed)
    start_time = current_time

# Cleanup
vis.Close()