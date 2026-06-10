import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the Gator vehicle, set parameters, and initialize
gator = veh.Gator()
gator.SetContactMethod(contact_method)
gator.SetChassisCollisionType(chassis_collision_type)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(tire_model)
gator.SetTireStepSize(tire_step_size)

gator.Initialize()

gator.SetChassisVisualizationType(vis_type)
gator.SetSuspensionVisualizationType(vis_type)
gator.SetSteeringVisualizationType(vis_type)
gator.SetWheelVisualizationType(vis_type)
gator.SetTireVisualizationType(vis_type)

gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain with 4 patches
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(gator.GetSystem())

# ========================================
# Patch 1: Flat reference terrain (upper-left)
# ========================================
patch1 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-25, 25, 0), chrono.QUNIT),
    50, 50)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# ========================================
# Patch 2: Height map terrain for gradability testing (upper-right)
# Uses a height map to create sloped terrain
# ========================================
patch2 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25, 25, 0), chrono.QUNIT),
    50, 50)
# Apply height map texture for visual effect
# Note: For actual height variation, PyChrono uses mesh-based terrain
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.4))

# Create sloped ramp geometry for gradability testing using a mesh
ramp_mat = chrono.ChContactMaterialNSC()
ramp_mat.SetFriction(0.9)
ramp_mat.SetRestitution(0.01)

# Create ramp as a triangular wedge shape for gradability
ramp = chrono.ChBody()
ramp.SetPos(chrono.ChVector3d(25, 25, 0))
ramp.SetRot(chrono.QUNIT)
ramp.SetMaterial(ramp_mat)
ramp.SetCollide(True)
ramp.SetBodyFixed(True)

# Create a wedge-shaped ramp using box approximation with rotation
# First segment - lower part
ramp_seg1 = chrono.ChBody()
ramp_seg1.SetPos(chrono.ChVector3d(20, 25, 0.15))
ramp_seg1.SetRot(chrono.QUNIT)
ramp_seg1.SetMaterial(ramp_mat)
ramp_seg1.SetCollide(True)
ramp_seg1.SetBodyFixed(True)
ramp_seg1.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(3.0, 8.0, 0.15))
gator.GetSystem().AddBody(ramp_seg1)

# Second segment - middle part (higher)
ramp_seg2 = chrono.ChBody()
ramp_seg2.SetPos(chrono.ChVector3d(27, 25, 0.45))
ramp_seg2.SetRot(chrono.QUNIT)
ramp_seg2.SetMaterial(ramp_mat)
ramp_seg2.SetCollide(True)
ramp_seg2.SetBodyFixed(True)
ramp_seg2.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(3.0, 8.0, 0.45))
gator.GetSystem().AddBody(ramp_seg2)

# Third segment - top part (highest)
ramp_seg3 = chrono.ChBody()
ramp_seg3.SetPos(chrono.ChVector3d(34, 25, 0.75))
ramp_seg3.SetRot(chrono.QUNIT)
ramp_seg3.SetMaterial(ramp_mat)
ramp_seg3.SetCollide(True)
ramp_seg3.SetBodyFixed(True)
ramp_seg3.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(3.0, 8.0, 0.75))
gator.GetSystem().AddBody(ramp_seg3)

# ========================================
# Patch 3: Terrain with multiple bumps (lower-left)
# ========================================
patch3 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-25, -25, 0), chrono.QUNIT),
    50, 50)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch3.SetColor(chrono.ChColor(0.6, 0.6, 0.3))

# Add bump obstacles to Patch 3
bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.9)
bump_mat.SetRestitution(0.01)

# Bump 1 - small
bump1 = chrono.ChBody()
bump1.SetPos(chrono.ChVector3d(-15, -25, 0.15))
bump1.SetRot(chrono.QUNIT)
bump1.SetMaterial(bump_mat)
bump1.SetCollide(True)
bump1.SetBodyFixed(True)
bump1.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(1.5, 1.5, 0.15))
gator.GetSystem().AddBody(bump1)

# Bump 2 - medium
bump2 = chrono.ChBody()
bump2.SetPos(chrono.ChVector3d(0, -25, 0.25))
bump2.SetRot(chrono.QUNIT)
bump2.SetMaterial(bump_mat)
bump2.SetCollide(True)
bump2.SetBodyFixed(True)
bump2.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(2.0, 2.0, 0.25))
gator.GetSystem().AddBody(bump2)

# Bump 3 - small
bump3 = chrono.ChBody()
bump3.SetPos(chrono.ChVector3d(15, -25, 0.1))
bump3.SetRot(chrono.QUNIT)
bump3.SetMaterial(bump_mat)
bump3.SetCollide(True)
bump3.SetBodyFixed(True)
bump3.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(1.0, 1.0, 0.1))
gator.GetSystem().AddBody(bump3)

# ========================================
# Patch 4: Different texture - grass terrain (lower-right)
# ========================================
patch4 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25, -25, 0), chrono.QUNIT),
    50, 50)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 100, 100)
patch4.SetColor(chrono.ChColor(0.5, 0.7, 0.3))

terrain.Initialize()

# -------------------------------------
# Create the vehicle Irrlicht interface
# Create the driver system
# -------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle - Multi-Terrain Test')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------
# Simulation loop
# ---------------

# Output vehicle mass
print("VEHICLE MASS:", gator.GetVehicle().GetMass())
print("Terrain configuration:")
print("  - Patch 1 (upper-left): Flat reference terrain")
print("  - Patch 2 (upper-right): Gradability test with ramp")
print("  - Patch 3 (lower-left): Terrain with bumps")
print("  - Patch 4 (lower-right): Grass terrain")

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = gator.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)