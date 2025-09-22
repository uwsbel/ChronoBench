# =============================================================================
# FEDA demo with first-person camera (PyChrono  –  core, vehicle, sensor)
# =============================================================================
import math
import pychrono.core     as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle  as veh
import pychrono.sensor   as sens          # << needed for sensors

# ----------------------------------------------------------------------------- 
# Chrono & vehicle data directories
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# ----------------------------------------------------------------------------- 
# Initial vehicle state
# -----------------------------------------------------------------------------
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.ChassisCollisionType_NONE   # << corrected enum
tire_model             = veh.TireModelType_TMEASY

# ----------------------------------------------------------------------------- 
# Terrain
# -----------------------------------------------------------------------------
terrainHeight = 0
terrainLength = 100.0
terrainWidth  = 100.0

# ----------------------------------------------------------------------------- 
# Other simulation parameters
# -----------------------------------------------------------------------------
contact_method    = chrono.ChContactMethod_NSC
step_size         = 1e-3
tire_step_size    = step_size
render_step_size  = 1.0 / 50.0                      # 50 FPS

# =============================================================================
# 1.  CREATE THE VEHICLE
# =============================================================================
vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Visualisation modes
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Use BULLET collision engine
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# =============================================================================
# 2.  CREATE THE TERRAIN  (with grass texture)
# =============================================================================
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # << grass
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# =============================================================================
# 3.  IRRLICHT VISUAL SYSTEM (for interactive driver)
# =============================================================================
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA vehicle")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# =============================================================================
# 4.  SENSOR MANAGER, LIGHTS, FIRST-PERSON CAMERA
# =============================================================================
mgr = sens.ChSensorManager(vehicle.GetSystem())

# Point lights – brighten the whole scene for the camera
mgr.scene.AddPointLight(chrono.ChVectorF( 10,   0, 10), chrono.ChColor(1, 1, 1), 600)
mgr.scene.AddPointLight(chrono.ChVectorF(-10,   0, 10), chrono.ChColor(1, 1, 1), 600)

# Camera parameters
cam_width  = 1920          # high resolution
cam_height = 1080
fov_deg    = 80            # reasonable wide angle
fov_rad    = math.radians(fov_deg)

# Place camera at the driver’s approximate eye position w.r.t chassis COM
cam_offset = chrono.ChFrameD(chrono.ChVector3d(0.5, 0.0, 1.2), chrono.QUNIT)

chassis_body = vehicle.GetVehicle().GetChassisBody()

camera = sens.ChCameraSensor(
    chassis_body,              # parent body
    update_rate=1/render_step_size,  # same rate as graphics frames (50 Hz)
    offset_pose=cam_offset,
    width=cam_width,
    height=cam_height,
    fov=fov_rad,
)

# Add a visualization (OpenGL) filter so the image pops up on screen
camera.PushFilter(sens.ChFilterVisualize(cam_width, cam_height, "On-board camera"))

# Finally add camera to the manager
mgr.AddSensor(camera)

# =============================================================================
# 5.  DRIVER
# =============================================================================
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Quick confirmation
print("VEHICLE MASS :", vehicle.GetVehicle().GetMass())

# =============================================================================
# 6.  SIMULATION LOOP
# =============================================================================
render_steps    = math.ceil(render_step_size / step_size)
realtime_timer  = chrono.ChRealtimeStepTimer()
step_number     = 0
render_frame    = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # -------------------------------------------------------------------------
    # Render with Irrlicht every 'render_step_size'
    # -------------------------------------------------------------------------
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # -------------------------------------------------------------------------
    # Collect inputs & synchronize modules
    # -------------------------------------------------------------------------
    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # -------------------------------------------------------------------------
    # Advance the dynamics
    # -------------------------------------------------------------------------
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # -------------------------------------------------------------------------
    # Update all sensors (camera, etc.)
    # -------------------------------------------------------------------------
    mgr.Update()                       # << mandatory for camera images

    # -------------------------------------------------------------------------
    # House-keeping
    # -------------------------------------------------------------------------
    step_number += 1
    realtime_timer.Spin(step_size)