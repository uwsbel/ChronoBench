# ==============================================================================
#  MODIFIED PyChrono script – MAN 10t vehicle demo
#
#  Changes made:
#   • Imported pychrono.sensor and numpy.
#   • Added a sensor manager and a lidar sensor attached to the chassis.
#   • Added 20 random boxes (with random position and size) to the scene.
#   • Changed the terrain texture from "tile4.jpg" to "grass.jpg".
#   • Updated the sensor manager inside the simulation loop.
# ==============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as psensor
import numpy as np
import math

# -----------------------------------------------------------------------------
#  Chrono & vehicle data paths
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# -----------------------------------------------------------------------------
#  Simulation parameters
# -----------------------------------------------------------------------------
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

terrainHeight = 0.0
terrainLength = 100.0
terrainWidth = 100.0

trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

contact_method = chrono.ChContactMethod_NSC
contact_vis = False

step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50.0   # 50 FPS

# -----------------------------------------------------------------------------
#  Create the vehicle (MAN‑10t)
# -----------------------------------------------------------------------------
vehicle = veh.MAN_10t()
vehicle.SetContactMethod(contact_method)
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

# Use the BULLET collision system (as in the original script)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -----------------------------------------------------------------------------
#  Rigid terrain
# -----------------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth)

# Changed texture to grass.jpg as requested
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# -----------------------------------------------------------------------------
#  Irrlicht visualization
# -----------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# -----------------------------------------------------------------------------
#  Driver system
# -----------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# -----------------------------------------------------------------------------
#  Print vehicle mass
# -----------------------------------------------------------------------------
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# -----------------------------------------------------------------------------
#  Sensor manager & lidar sensor
# -----------------------------------------------------------------------------
sensor_manager = psensor.ChSensorManager(vehicle.GetSystem())

# Get the chassis body (used as the parent for the lidar)
chassis = vehicle.GetVehicle().GetChassis()

# Define lidar geometry (360° horizontal, 60° vertical, 50 m range)
lidar_geo = psensor.ChLidarSensorGeometry()
lidar_geo.HorizontalFov = 2.0 * math.pi          # 360°
lidar_geo.VerticalFov = math.pi / 3.0            # 60°
lidar_geo.MaxDistance = 50.0                    # 50 m
lidar_geo.SampleRadius = 2                     # sampling radius

# Use default intrinsics
lidar_intri = psensor.ChLidarSensorIntrinsics()

# Create the lidar sensor
lidar = psensor.ChLidarSensor(
    chassis,          # parent body
    30.0,             # update rate (Hz)
    lidar_geo,
    lidar_intri)
lidar.SetName("Lidar")

# Add lidar to the manager
sensor_manager.AddSensor(lidar)

# -----------------------------------------------------------------------------
#  Random boxes (added to the simulation)
# -----------------------------------------------------------------------------
np.random.seed(42)                       # reproducibility
num_boxes = 20

# Material for the boxes
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.0)

for i in range(num_boxes):
    # Random position (x, y) in [-20, 20] and height in [0.5, 3.0]
    x = np.random.uniform(-20.0, 20.0)
    y = np.random.uniform(-20.0, 20.0)
    z = np.random.uniform(0.5, 3.0)

    # Random dimensions (full side lengths)
    dx = np.random.uniform(0.2, 1.0)
    dy = np.random.uniform(0.2, 1.0)
    dz = np.random.uniform(0.2, 1.0)

    # Create a box body (density = 1000 kg/m³)
    box = chrono.ChBodyEasyBox(dx, dy, dz, 1000.0, box_mat)
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetBodyFixed(False)

    # Visual appearance (brownish colour)
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.5, 0.2))

    # Add to the system
    vehicle.GetSystem().AddBody(box)

# -----------------------------------------------------------------------------
#  Simulation loop
# -----------------------------------------------------------------------------
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render the scene (as in the original script)
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Retrieve driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronise all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update sensor manager (lidar will be updated here)
    sensor_manager.Update(time)

    step_number += 1

    # Keep real‑time synchronisation
    realtime_timer.Spin(step_size)