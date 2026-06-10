# PyChrono Gator vehicle simulation with rigid terrain, interactive driver,
# Irrlicht visualization, and a chassis-mounted Chrono::Sensor camera.

import os
import math

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# ---------------------------------------------------------------------
# Version-compatible aliases for common Chrono types
# ---------------------------------------------------------------------
ChVector = getattr(chrono, "ChVector3d", chrono.ChVectorD)
ChVectorF = getattr(chrono, "ChVector3f", chrono.ChVectorF)
ChCoordsys = getattr(chrono, "ChCoordsysd", chrono.ChCoordsysD)
ChFrame = getattr(chrono, "ChFramed", chrono.ChFrameD)

# ---------------------------------------------------------------------
# Data paths
# ---------------------------------------------------------------------
# If CHRONO_DATA_DIR is defined, use it. Otherwise PyChrono's default
# installed data path will be used.
if "CHRONO_DATA_DIR" in os.environ:
    chrono.SetChronoDataPath(os.environ["CHRONO_DATA_DIR"])

veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# ---------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------
step_size = 1.0e-3
tire_step_size = 1.0e-3

render_fps = 60
render_step = 1.0 / render_fps
render_steps = int(math.ceil(render_step / step_size))

sensor_update_rate = 30.0
camera_width = 1280
camera_height = 720
camera_fov = 1.408  # radians, about 80 deg

end_time = 120.0

# Initial vehicle pose
init_loc = ChVector(0.0, 0.0, 0.45)
init_rot = chrono.QUNIT

# ---------------------------------------------------------------------
# Create and initialize the Gator vehicle
# ---------------------------------------------------------------------
gator = veh.Gator()

gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(ChCoordsys(init_loc, init_rot))

# Use collision only where desired.
gator.SetChassisCollisionType(veh.CollisionType_NONE)

# Tire model, if supported by the installed Chrono version.
try:
    gator.SetTireType(veh.TireModelType_RIGID)
except Exception:
    pass

try:
    gator.SetTireStepSize(tire_step_size)
except Exception:
    pass

# Set different visualization types for different parts.
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

gator.Initialize()

system = gator.GetSystem()
system.SetGravitationalAcceleration(ChVector(0.0, 0.0, -9.81))

# ---------------------------------------------------------------------
# Rigid terrain
# ---------------------------------------------------------------------
terrain = veh.RigidTerrain(system)

try:
    terrain_mat = chrono.ChContactMaterialNSC()
except Exception:
    terrain_mat = chrono.ChMaterialSurfaceNSC()

terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    terrain_mat,
    ChCoordsys(ChVector(0.0, 0.0, 0.0), chrono.QUNIT),
    200.0,
    200.0,
)

patch.SetColor(chrono.ChColor(0.35, 0.45, 0.25))

try:
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
except Exception:
    pass

terrain.Initialize()

# ---------------------------------------------------------------------
# Irrlicht visualization system
# ---------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono Gator Vehicle with Sensors")
vis.SetWindowSize(1280, 720)

# Chase camera target point in the vehicle frame.
vis.SetChaseCamera(ChVector(0.0, 0.0, 0.75), 6.0, 0.5)

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(gator.GetVehicle())

# ---------------------------------------------------------------------
# Interactive keyboard driver
# ---------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step / steering_time)
driver.SetThrottleDelta(render_step / throttle_time)
driver.SetBrakingDelta(render_step / braking_time)

driver.Initialize()

print("Interactive driver controls:")
print("  W / Up Arrow    : throttle")
print("  S / Down Arrow  : brake")
print("  A / Left Arrow  : steer left")
print("  D / Right Arrow : steer right")

# ---------------------------------------------------------------------
# Sensor manager, point lights, and chassis-mounted camera
# ---------------------------------------------------------------------
manager = sens.ChSensorManager(system)

# Add lighting for the sensor-rendered scene.
manager.scene.AddPointLight(
    ChVectorF(8.0, -6.0, 8.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)

manager.scene.AddPointLight(
    ChVectorF(-8.0, 6.0, 6.0),
    chrono.ChColor(0.6, 0.7, 1.0),
    300.0,
)

# Attach camera to the vehicle chassis.
try:
    chassis_body = gator.GetChassisBody()
except Exception:
    chassis_body = gator.GetVehicle().GetChassisBody()

# Camera offset in the chassis frame.
# Chrono vehicle frames use X forward, Y left, Z up. With identity rotation,
# the Chrono sensor camera looks forward from the chassis.
camera_offset_pose = ChFrame(
    ChVector(0.6, 0.0, 1.25),
    chrono.QUNIT,
)

camera = sens.ChCameraSensor(
    chassis_body,
    sensor_update_rate,
    camera_offset_pose,
    camera_width,
    camera_height,
    camera_fov,
)

camera.SetName("Gator chassis camera")

# Visualize rendered camera images in a separate window.
camera.PushFilter(
    sens.ChFilterVisualize(
        camera_width,
        camera_height,
        "Gator Chassis Camera",
    )
)

# Provide CPU access to RGBA images if further processing is desired.
camera.PushFilter(sens.ChFilterRGBA8Access())

# Optional: uncomment to save camera images.
# camera.PushFilter(sens.ChFilterSave("./camera_output/"))

manager.AddSensor(camera)

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = system.GetChTime()

    if time >= end_time:
        break

    # Get current driver inputs.
    driver_inputs = driver.GetInputs()

    # Synchronize all modules.
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Update sensor manager. This renders sensor data at each sensor's
    # specified update rate.
    manager.Update()

    # Render Irrlicht visualization.
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Advance all modules.
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    # Real-time pacing.
    realtime_timer.Spin(step_size)

    step_number += 1