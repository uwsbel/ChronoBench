import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

# ---------------------------------------------------------------------
# Chrono initialisation
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# ---------------------------------------------------------------------
# General simulation parameters
# ---------------------------------------------------------------------
contact_method = chrono.ChContactMethod_NSC
step_size      = 1e-3
render_fps     = 50
render_step    = 1.0 / render_fps

# ---------------------------------------------------------------------
# Vehicle parameters
# ---------------------------------------------------------------------
initLoc  = chrono.ChVector3d(0, 0, 0.5)
initRot  = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.ChassisCollisionType_NONE        # <- fixed enumeration
tire_model             = veh.TireModelType_TMEASY
tire_step_size         = step_size

# ---------------------------------------------------------------------
# Create vehicle
# ---------------------------------------------------------------------
gator = veh.Gator()
gator.SetContactMethod(contact_method)
gator.SetChassisCollisionType(chassis_collision_type)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(tire_model)
gator.SetTireStepSize(tire_step_size)
gator.Initialize()

# visualisation options ------------------------------------------------
gator.SetChassisVisualizationType(vis_type)
gator.SetSuspensionVisualizationType(vis_type)
gator.SetSteeringVisualizationType(vis_type)
gator.SetWheelVisualizationType(vis_type)
gator.SetTireVisualizationType(vis_type)

# ---------------------------------------------------------------------
# Collision system
# ---------------------------------------------------------------------
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# =====================================================================
# Terrain – FOUR different patches + one height-map + bumps
# =====================================================================
system  = gator.GetSystem()
terrain = veh.RigidTerrain(system)

# Common contact material for all patches
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Size of each individual patch
patch_len = 50.0        # x size
patch_wid = 50.0        # y size

# Offsets so that the patches form a 2 × 2 carpet
offset = patch_len * 0.5

# ---------------- flat grass patch ----------------
p1_center = chrono.ChVector3d(-offset, -offset, 0.0)
p1 = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(p1_center, chrono.QUNIT),
        patch_len, patch_wid)
p1.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 20, 20)

# ---------------- flat dirt patch ----------------
p2_center = chrono.ChVector3d( offset, -offset, 0.0)
p2 = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(p2_center, chrono.QUNIT),
        patch_len, patch_wid)
p2.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

# ---------------- flat rocky patch ----------------
p3_center = chrono.ChVector3d(-offset,  offset, 0.0)
p3 = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(p3_center, chrono.QUNIT),
        patch_len, patch_wid)
p3.SetTexture(veh.GetDataFile("terrain/textures/rocks.jpg"), 10, 10)

# ---------------- height-map patch ----------------
# A small height-map (*.bmp or *.png) comes with the Chrono data files.
# Scale in X and Y equals patch_len / patch_wid
# Height scale = 3.0 m (vertical exaggeration)
hmap_center = chrono.ChVector3d( offset,  offset, 0.0)
heightmap_file = veh.GetDataFile("terrain/height_maps/bump64.png")   # make sure it exists
p4 = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(hmap_center, chrono.QUNIT),
        heightmap_file,
        patch_len, patch_wid,      # size in X, size in Y
        3.0)                       # height scaling (max height ≈ 3 m)
p4.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 10, 10)

# ---------------------------------------------------------------------
# Add a “bump’’ (shallow box) at the centre of every patch
# ---------------------------------------------------------------------
def add_bump(center_pos, width=1.5, height=0.25, depth=2.0):
    bump = chrono.ChBodyEasyBox(width, height, depth, 1000, True, True)
    bump.SetPos(center_pos + chrono.ChVector3d(0, 0, height/2))
    bump.SetBodyFixed(True)
    bump.SetCollide(True)
    system.Add(bump)

for c in [p1_center, p2_center, p3_center, hmap_center]:
    add_bump(c)

terrain.Initialize()

# ---------------------------------------------------------------------
# Irrlicht visualisation
# ---------------------------------------------------------------------
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator vehicle – 4 terrain patches")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())

# ---------------------------------------------------------------------
# Interactive driver
# ---------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0     # sec to go -1 … +1
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step / steering_time)
driver.SetThrottleDelta(render_step / throttle_time)
driver.SetBrakingDelta(render_step  / braking_time)
driver.Initialize()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
print("VEHICLE MASS:", gator.GetVehicle().GetMass())

render_steps   = math.ceil(render_step / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
render_frame   = 0

while vis.Run():
    time = system.GetChTime()

    # Render -----------------------------------------------------------
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Collect driver inputs -------------------------------------------
    driver_inputs = driver.GetInputs()

    # Synchronise modules ---------------------------------------------
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance ----------------------------------------------------------
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)