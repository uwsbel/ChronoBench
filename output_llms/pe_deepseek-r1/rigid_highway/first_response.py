import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# =======================
# 1. SYSTEM INITIALIZATION
# =======================
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# =======================
# 2. VEHICLE SETUP
# =======================
# Initialize HMMWV with specific parameters
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 0, 0.5),  # Initial position
    chrono.Q_from_AngZ(0)           # Initial orientation
))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(0.001)
hmmwv.Initialize()

# Configure visualization
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# Get vehicle systems
vehicle = hmmwv.GetVehicle()
tires = [hmmwv.GetTire(i) for i in range(4)]

# =======================
# 3. CUSTOM TERRAIN SETUP
# =======================
# Create mesh terrain
terrain = veh.RigidTerrain(system)

# Mesh materials
terrain_material = chrono.ChContactMaterialNSC()
terrain_material.SetFriction(0.9)
terrain_material.SetRestitution(0.01)

# Load collision and visual meshes
mesh_path = os.path.join(chrono.GetChronoDataPath(), "models", "highway")
collision_mesh = os.path.join(mesh_path, "Highway_col.obj")
visual_mesh = os.path.join(mesh_path, "Highway_vis.obj")

patch = terrain.AddPatch(
    terrain_material,
    chrono.ChCoordsysd(),  # Default position
    collision_mesh,
    visual_mesh,
    0.01  # Mesh sweep sphere radius
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
terrain.Initialize()

# =======================
# 4. VISUALIZATION SYSTEM
# =======================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on Custom Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, 6, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# Attach vehicle systems
vis.AddTypicalLights()
hmmwv.GetVehicle().AddVisualSystem(vis)

# =======================
# 5. INTERACTIVE DRIVER
# =======================
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# =======================
# 6. SIMULATION LOOP
# =======================
step_size = 0.002
frame_rate = 50  # 50 FPS
frame_step = 1.0 / frame_rate
time = 0
frame_count = 0

while vis.Run():
    time = system.GetChTime()
    
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    # Update vehicle
    hmmwv.Synchronize(time, driver_inputs, terrain)
    
    # Advance simulation
    driver.Advance(step_size)
    hmmwv.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    # Update visualization at specified frame rate
    if time >= frame_count * frame_step:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame_count += 1

# =======================
# 7. POST-SIMULATION CLEANUP
# =======================
vis.GetDevice().closeDevice()