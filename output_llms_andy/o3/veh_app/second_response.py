# =============================================================================
# Chrono::Sensor + Vehicle demo
# - Gator starts at a new initial location
# - One cubic and one cylindrical fixed obstacle are added
# - A camera  and a LiDAR sensor (with several post–processing filters) are
#   mounted on the vehicle
# - Simple driver that keeps a constant steering and throttle input
# =============================================================================

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as irr           # (not used, but keep in case you want 3-D interactive view)
import pychrono.vehicle as veh
import pychrono.sensor as sens

# ----------------------------------------------------------------------------- 
# Chrono data directories
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataFilepath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

print("Using vehicle data folder:", chrono.GetChronoDataPath() + 'vehicle/')

# =============================================================================
# 1.  Vehicle creation and initialisation
# =============================================================================
initLoc = chrono.ChVector3d(0, -5, 0.4)          # *** CHANGED per instructions ***
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# visualisation types ---------------------------------------------------------
chassis_vis_type     = veh.VisualizationType_MESH
suspension_vis_type  = veh.VisualizationType_PRIMITIVES
steering_vis_type    = veh.VisualizationType_PRIMITIVES
wheel_vis_type       = veh.VisualizationType_NONE
tire_vis_type        = veh.VisualizationType_MESH

# simulation control ----------------------------------------------------------
step_size       = 1e-3
tire_step_size  = step_size
render_step     = 1.0/50.0          # 50 FPS
end_time        = 30                # [s]

# -----------------------------------------------------------------------------
# build the vehicle
# -----------------------------------------------------------------------------
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)

# some information ------------------------------------------------------------
print("Vehicle mass      :", gator.GetVehicle().GetMass())
print("Driveline type    :", gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type        :", gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type         :", gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print()

# use Bullet collision system -------------------------------------------------
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

system = gator.GetSystem()

# =============================================================================
# 2.  Rigid–terrain definition
# =============================================================================
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# the helper constant CSYSNORM is not available from Python, create an explicit
#                 (pos,                           rot)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    50, 50
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# =============================================================================
# 3.  Additional obstacles (box + cylinder) – both coloured blue
# =============================================================================
blue = chrono.ChColor(0.1, 0.2, 0.9)

# --- box (1 x 1 x 1) ---------------------------------------------------------
box        = chrono.ChBodyEasyBox(1, 1, 1,         # size x,y,z
                                  1000,            # density
                                  True,            # visualization
                                  True)            # collision
box.SetBodyFixed(True)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.GetVisualShape(0).SetColor(blue)
system.Add(box)

# --- cylinder (radius 0.5, height 1) ----------------------------------------
cyl         = chrono.ChBodyEasyCylinder(0.5, 1.0,  # radius, height
                                        1000,
                                        True,
                                        True)
cyl.SetBodyFixed(True)
cyl.SetPos(chrono.ChVector3d(0, 0, 1.5))
cyl.GetVisualShape(0).SetColor(blue)
system.Add(cyl)

# =============================================================================
# 4.  Driver
# =============================================================================
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# =============================================================================
# 5.  Sensor manager and sensors
# =============================================================================
update_rate        = 10                      # Hz (used by all sensors)
image_width        = 1280
image_height       = 720
fov                = 1.408                  # camera FOV

manager = sens.ChSensorManager(system)

# a single point light for the whole scene
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0
)

# ------------------------------------------------ camera (already in original)
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-8.0, 0.0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
)

cam = sens.ChCameraSensor(
    gator.GetChassisBody(),      # parent body
    update_rate,                 # Hz
    cam_offset,                  # (relative) pose
    image_width,
    image_height,
    fov                          # HFOV
)
cam.SetName("Third-person camera")
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)

# ------------------------------------------------ LiDAR  ( *** NEW *** )
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 2),      # 2 m above chassis origin
    chrono.QUNIT
)

lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),    # parent body
    update_rate,               # scan rate [Hz]
    lidar_offset,              # pose wrt parent
    800,                       # horizontal samples
    300,                       # vertical channels
    2 * math.pi,               # horizontal FOV  (360°)
    100.0                      # max range [m]
)

# vertical FOV
lidar.SetVerticalFOVUpper( math.pi / 12.0)   # +15°
lidar.SetVerticalFOVLower(-math.pi / 6.0)    # –30°

# beam characteristics
lidar.SetBeamShape(sens.LidarBeamShape.RECTANGULAR)
lidar.SetSampleRadius(2)
lidar.SetDivergenceAngle(0.003)
lidar.SetReturnMode(sens.LidarReturnMode.STRONGEST_RETURN)

lidar.SetName("Roof-mounted LiDAR")

# ----- filters
lidar.PushFilter(sens.ChFilterLidarDepth())
lidar.PushFilter(sens.ChFilterLidarIntensity())
lidar.PushFilter(sens.ChFilterXYZIPointCloud())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, "LiDAR Point Cloud"))

manager.AddSensor(lidar)

# =============================================================================
# 6.  Simulation loop
# =============================================================================
realtime_timer = chrono.ChRealtimeStepTimer()
time = 0.0

while time < end_time:
    time = system.GetChTime()

    # ------------------------------------------------ driver inputs
    # ( *** values changed as required – already 0.5 & 0.2 *** )
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)

    inputs = driver.GetInputs()

    # ------------------------------------------------ module synchronisation
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, inputs, terrain)

    # ------------------------------------------------ update sensors
    manager.Update()

    # ------------------------------------------------ advance dynamics
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    realtime_timer.Spin(step_size)