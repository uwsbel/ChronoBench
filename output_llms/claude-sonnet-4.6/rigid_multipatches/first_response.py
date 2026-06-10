import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# =============================================================================
# Initialize PyChrono environment
# =============================================================================
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# =============================================================================
# HMMWV Vehicle Initialization
# =============================================================================
# Initial vehicle position and orientation
initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for all vehicle components
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model
tire_model = veh.TireModelType_RIGID

# =============================================================================
# Create the HMMWV vehicle
# =============================================================================
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(chassis_collision_type)
hmmwv.SetChassisFixed(False)

# Set initial position and orientation
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))

# Engine and drivetrain type
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)

# Steering type
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)

# Tire model
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(1e-3)

# Initialize vehicle
hmmwv.Initialize()

# Apply mesh visualization to all vehicle components
hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)

# =============================================================================
# Create the terrain system
# =============================================================================
terrain = veh.RigidTerrain(hmmwv.GetSystem())

# --- Patch 1: Flat patch with asphalt texture ---
patch1_mat = chrono.ChContactMaterialNSC()
patch1_mat.SetFriction(0.9)
patch1_mat.SetRestitution(0.01)

patch1 = terrain.AddPatch(
    patch1_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    40, 40
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# --- Patch 2: Flat patch with grass texture ---
patch2_mat = chrono.ChContactMaterialNSC()
patch2_mat.SetFriction(0.8)
patch2_mat.SetRestitution(0.01)

patch2 = terrain.AddPatch(
    patch2_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(50, 0, 0), chrono.QUNIT),
    40, 40
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 20, 20)
patch2.SetColor(chrono.ChColor(0.4, 0.7, 0.4))

# --- Patch 3: Flat patch with dirt texture ---
patch3_mat = chrono.ChContactMaterialNSC()
patch3_mat.SetFriction(0.7)
patch3_mat.SetRestitution(0.01)

patch3 = terrain.AddPatch(
    patch3_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-50, 0, 0), chrono.QUNIT),
    40, 40
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 20)
patch3.SetColor(chrono.ChColor(0.6, 0.4, 0.2))

# --- Patch 4: Mesh-based patch for bump ---
patch4_mat = chrono.ChContactMaterialNSC()
patch4_mat.SetFriction(0.9)
patch4_mat.SetRestitution(0.01)

patch4 = terrain.AddPatch(
    patch4_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 50, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"),
    True, 0.01
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.8))

# --- Patch 5: Heightmap-based patch ---
patch5_mat = chrono.ChContactMaterialNSC()
patch5_mat.SetFriction(0.8)
patch5_mat.SetRestitution(0.01)

patch5 = terrain.AddPatch(
    patch5_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -50, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/test64.bmp"),
    40, 40, 0, 3
)
patch5.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 20, 20)
patch5.SetColor(chrono.ChColor(0.6, 0.8, 0.6))

# Initialize terrain
terrain.Initialize()

# =============================================================================
# Create the interactive driver system
# =============================================================================
driver = veh.ChInteractiveDriverIRR(vis := irr.ChVisualSystemIrrlicht())

# =============================================================================
# Create Irrlicht visualization
# =============================================================================
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(hmmwv.GetSystem())
vis.SetWindowTitle('HMMWV on Complex Rigid Terrain')
vis.SetWindowSize(1280, 720)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-6, -6, 2), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(10, -6, 100),
    chrono.ChVector3d(0, 0, 0),
    200, 5, 200, 35
)

# =============================================================================
# Create driver system (re-create after vis is properly initialized)
# =============================================================================
driver = veh.ChInteractiveDriverIRR(vis)

# Set driver input gains
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)

# Initialize driver
driver.Initialize()

# =============================================================================
# Simulation settings
# =============================================================================
# Simulation step size
step_size = 1e-3
tire_step_size = 1e-3

# Render frame rate
render_step_size = 1.0 / 50  # 50 FPS

# Simulation end time
t_end = 1000.0  # Run until user quits

# =============================================================================
# Output vehicle mass
# =============================================================================
print("Vehicle mass: ", hmmwv.GetVehicle().GetMass())

# =============================================================================
# Initialize simulation frame counter
# =============================================================================
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# =============================================================================
# Simulation loop
# =============================================================================
while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    # Render scene
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    # Update camera to follow vehicle
    vehicle_pos = hmmwv.GetVehicle().GetPos()
    vehicle_fwd = hmmwv.GetVehicle().GetForwardVector()

    cam_pos = vehicle_pos - vehicle_fwd * 8 + chrono.ChVector3d(0, 0, 3)
    cam_target = vehicle_pos + chrono.ChVector3d(0, 0, 1)
    vis.UpdateCamera(cam_pos, cam_target)

    # Increment step counter
    step_number += 1

    # Enforce real-time execution
    realtime_timer.Spin(step_size)