import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os

# ------------------------------------------------------------------
# 1) Set data paths
# ------------------------------------------------------------------
# (you may need to set the CHRONO_DATA_PATH env var or adjust this)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle'))

# ------------------------------------------------------------------
# 2) Initial vehicle location and orientation
# ------------------------------------------------------------------
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# ------------------------------------------------------------------
# 3) Visualization / collision parameters
# ------------------------------------------------------------------
vis_type        = veh.VisualizationType_PRIMITIVES
vis_type_mesh   = veh.VisualizationType_MESH
chassis_col     = veh.CollisionType_NONE

# ------------------------------------------------------------------
# 4) Tire model choice
#    Changed from TMEASY to PACEJKA (’89 version)
# ------------------------------------------------------------------
tire_model       = veh.TireModelType_PACEJKA

# ------------------------------------------------------------------
# 5) Terrain parameters
# ------------------------------------------------------------------
terrainHeight = 0      # Z = 0
terrainLength = 100.0
terrainWidth  = 100.0

# ------------------------------------------------------------------
# 6) Simulation parameters
# ------------------------------------------------------------------
contact_method   = chrono.ChContactMethod_NSC
step_size        = 5e-4     # was 1e-3, now reduced
tire_step_size   = step_size
render_fps       = 50
render_step_size = 1.0 / render_fps

# ------------------------------------------------------------------
# 7) Create the vehicle
# ------------------------------------------------------------------
vehicle = veh.CityBus()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_col)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))

vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Visualization types
vehicle.SetChassisVisualizationType(vis_type_mesh)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type_mesh)
vehicle.SetTireVisualizationType(vis_type_mesh)

# Use Bullet for contact
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Print out mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# ------------------------------------------------------------------
# 8) Create the terrain
# ------------------------------------------------------------------
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength,
    terrainWidth)

# Changed texture to dirt.jpg
patch.SetTexture(
    veh.GetDataFile("terrain/textures/dirt.jpg"),
    200, 200)

patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ------------------------------------------------------------------
# 9) Visualization (Irrlicht) setup
# ------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('City Bus Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(-15, 10, 5.8), 6.0, 3.5)
vis.Initialize()

# Note: use GetChronoDataFile for the logo
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# ------------------------------------------------------------------
# 10) Driver system
# ------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

# Steering / throttle / brake response times
steering_time = 1.0
throttle_time = 1.0
brake_time    = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / brake_time)
driver.Initialize()

# ------------------------------------------------------------------
# 11) Run the simulation
# ------------------------------------------------------------------
render_steps    = math.ceil(render_step_size / step_size)
realtime_timer  = chrono.ChRealtimeStepTimer()
step_number     = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    # Render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize all modules
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vehicle.Synchronize(t, driver_inputs, terrain)
    vis.Synchronize(t, driver_inputs)

    # Advance all modules by one step
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

tire_model = veh.TireModelType_PACEJKA

step_size      = 5e-4
  tire_step_size = step_size

patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)