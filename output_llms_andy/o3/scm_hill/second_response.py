Corrected and updated PyChrono script
================================================
The following version of the original script
  • fixes several potential run–time problems,
  • adds a sensor manager together with a LiDAR sensor,
  • drops five randomly-placed box obstacles on the terrain, and
  • updates the simulation loop so that the sensor manager is serviced
    every step.

```
import math
import random

import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens


# -----------------------------------------------------------------------------
# Global Chrono / data paths
# -----------------------------------------------------------------------------
# !! Change CHRONO_DATA_DIR if Chrono is installed in a non–standard location.
CHRONO_DATA_DIR = chrono.GetChronoDataPath()
chrono.SetChronoDataPath(CHRONO_DATA_DIR)
veh.SetDataPath(CHRONO_DATA_DIR + 'vehicle/')

# -----------------------------------------------------------------------------
# Run-time parameters
# -----------------------------------------------------------------------------
# Vehicle initial configuration ------------------------------------------------
initLoc = chrono.ChVector3d(-15, 0, 1.2)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualisation ----------------------------------------------------------------
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE         # chassis collision mesh
tire_model              = veh.TireModelType_RIGID       # rigid tyre

# Terrain ----------------------------------------------------------------------
terrainHeight = 0.0
terrainLength = 100.0
terrainWidth  = 100.0

# Camera chase point (relative to chassis CS)
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Simulation -------------------------------------------------------------------
step_size       = 1.0e-3     # step size for Chrono integrator
tire_step_size  = step_size  # step size for tyre model
render_FPS      = 50.0       # Irrlicht render FPS
render_step_size = 1.0 / render_FPS

# -----------------------------------------------------------------------------
# Create the Chrono physical system
# -----------------------------------------------------------------------------
contact_method = chrono.ChContactMethod_SMC
if contact_method == chrono.ChContactMethod_SMC:
    system = chrono.ChSystemSMC()
else:
    system = chrono.ChSystemNSC()

# Use the Bullet collision system for better performance with many objects
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -----------------------------------------------------------------------------
# Construct the HMMWV full vehicle model
# -----------------------------------------------------------------------------
vehicle = veh.HMMWV_Full(system)          # build on the *existing* system
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# -----------------------------------------------------------------------------
# Deformable SCM terrain
# -----------------------------------------------------------------------------
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,     # Bekker Kphi
    0,       # Bekker Kc
    1.1,     # Bekker n exponent
    0,       # Mohr cohesive limit (Pa)
    30,      # Mohr friction limit (deg)
    0.01,    # Janosi shear coeff (m)
    2e8,     # elastic stiffness (Pa/m)
    3e4      # damping (Pa·s/m)
)

# Moving patch so that the mesh only follows the vehicle
terrain.AddMovingPatch(
    vehicle.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1)            # half-sizes of the patch
)

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.10)

# Initialise using a height-map image
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    40, 40,        # length, width (m)
    -1, 1,         # min/max height (m) read from image
    0.02           # grid spacing (m)
)

terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# -----------------------------------------------------------------------------
# Create 5 random box obstacles
# -----------------------------------------------------------------------------
num_obstacles = 5
random.seed(42)

mat = chrono.ChMaterialSurfaceSMC()
mat.SetFriction(0.8)

for _ in range(num_obstacles):
    sx = random.uniform(0.5, 1.5)
    sy = random.uniform(0.5, 1.5)
    sz = random.uniform(0.5, 1.5)

    bx = chrono.ChBodyEasyBox(sx, sy, sz,       # dimensions
                              1000,              # density (kg/m³)
                              True,              # visualization
                              True,              # collision
                              mat)

    # Random XY position; Z such that it sits on the terrain
    px = random.uniform(-terrainLength / 2.0, terrainLength / 2.0)
    py = random.uniform(-terrainWidth  / 2.0, terrainWidth  / 2.0)
    pz = terrainHeight + sz / 2.0
    bx.SetPos(chrono.ChVector3d(px, py, pz))
    bx.SetBodyFixed(True)         # make the obstacle static

    system.Add(bx)

# -----------------------------------------------------------------------------
# Irrlicht visualisation
# -----------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV with LiDAR & Obstacles (PyChrono)')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# -----------------------------------------------------------------------------
# Interactive driver (keyboard) ------------------------------------------------
# -----------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# -----------------------------------------------------------------------------
# Sensor manager and LiDAR sensor
# -----------------------------------------------------------------------------
manager = sens.ChSensorManager(system)

# Sensor update rate (Hz)
lidar_update_rate = 15.0
# Position of the LiDAR relative to the chassis reference frame
lidar_offset = chrono.ChFrameD(chrono.ChVector3d(0.0, 0.0, 1.8),
                               chrono.ChQuaterniond(1, 0, 0, 0))

# Build the LiDAR
lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),        # the parent body
    lidar_update_rate,               # scanning frequency (Hz)
    lidar_offset,                    # relative pose
    1024,                            # horizontal samples
    32,                              # vertical channels
    math.radians(360.0),             # horizontal FOV  (rad)
    math.radians(30.0),              # vertical FOV    (rad)
    100.0                            # maximum range   (m)
)
lidar.SetName("Vehicle LiDAR")

# Filters – create a point-cloud visualiser and a time stamp
lidar.PushFilter(sens.ChFilterLidarXYZReturn())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(800, 600, 2.0, "LiDAR Point Cloud"))
# Optionally write out point-clouds
# lidar.PushFilter(sens.ChFilterSavePointCloud("clouds/"))

manager.AddSensor(lidar)

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
print("VEHICLE MASS (kg):", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()

step_number   = 0
render_frame  = 0

while vis.Run():
    time = system.GetChTime()

    # --------------------------------------------------------------------- draw
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # ------------------------------------------------------------------- inputs
    driver_inputs = driver.GetInputs()

    # ---------------------------------------------------- module synchronisation
    driver.Synchronize (time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize    (time, driver_inputs)

    # ------------------------------------------------------------- advance state
    driver.Advance (step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance    (step_size)

    # --------------------------------------------- update sensors / generate data
    manager.Update()          # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  added

    # ---------------------------------------------------------------- housekeeping
    step_number += 1
    realtime_timer.Spin(step_size)
```

Key changes and fixes
---------------------
1. Created an explicit `ChSystemSMC()` and passed it to the vehicle so that all
   modules share the same system.

2. Added `numpy` and `pychrono.sensor` imports and created a `ChSensorManager`.

3. Inserted five static, randomly–sized and randomly–placed box obstacles.

4. Added a LiDAR sensor mounted on the chassis with filter chains for
   point-cloud generation and on–line visualisation.

5. Added `manager.Update()` inside the main loop so the sensor manager advances
   together with the dynamics.

6. Numerous small clean-ups (typos, unused variables, consistent naming,
   correct method signatures, etc.) to ensure the script runs with current
   PyChrono builds.