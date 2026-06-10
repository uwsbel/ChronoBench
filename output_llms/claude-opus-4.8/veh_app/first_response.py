import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math

# -----------------------------------------------------------------------------
# 1. Initialize the PyChrono environment and core components
# -----------------------------------------------------------------------------

# Set the data path for vehicle and Chrono resources
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (options: NONE, PRIMITIVES, MESH)
chassis_vis_type    = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type   = veh.VisualizationType_PRIMITIVES
wheel_vis_type      = veh.VisualizationType_MESH
tire_vis_type       = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain dimensions
terrainHeight = 0
terrainLength = 100.0   # size in X direction
terrainWidth  = 100.0   # size in Y direction

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 2e-3
tire_step_size = 1e-3

# -----------------------------------------------------------------------------
# 2. Create and initialize the Gator vehicle
# -----------------------------------------------------------------------------

gator = veh.Gator()
gator.SetContactMethod(contact_method)
gator.SetChassisCollisionType(chassis_collision_type)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(tire_model)
gator.SetTireStepSize(tire_step_size)

# Initialize the vehicle and all systems
gator.Initialize()

# Set visualization types for the various vehicle parts
gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)

# Reference the underlying Chrono system
system = gator.GetSystem()

# -----------------------------------------------------------------------------
# 3. Create the rigid terrain
# -----------------------------------------------------------------------------

terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# -----------------------------------------------------------------------------
# 4. Create the interactive driver system
# -----------------------------------------------------------------------------

driver = veh.ChInteractiveDriverIRR  # placeholder reference; using vehicle driver
# Use the interactive driver attached to an Irrlicht visual system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AttachVehicle(gator.GetVehicle())

# Interactive driver (keyboard control)
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# -----------------------------------------------------------------------------
# 5. Create the sensor manager, lights, and camera
# -----------------------------------------------------------------------------

manager = sens.ChSensorManager(system)

# Add point lights to the scene
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(100, 100, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0)
manager.scene.AddPointLight(
    chrono.ChVector3f(-100, -100, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0)

# Camera parameters
update_rate = 30          # frames per second
image_width = 1280
image_height = 720
fov = 1.408               # horizontal field of view (radians)
lag = 0.0
exposure_time = 0.0

# Offset pose of the camera relative to the chassis
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0.0, 2.0),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))

# Create a camera attached to the vehicle chassis body
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),   # body to attach to
    update_rate,              # update rate in Hz
    offset_pose,              # offset pose
    image_width,              # image width
    image_height,             # image height
    fov                       # field of view
)
cam.SetName("Chassis Camera")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)

# Add filters to visualize and render the camera images
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())

# Add the camera to the sensor manager
manager.AddSensor(cam)

# -----------------------------------------------------------------------------
# 6. Run the simulation loop
# -----------------------------------------------------------------------------

# Output/initialization
gator.GetVehicle().EnableRealtime(True)

render_step_size = 1.0 / 50  # FPS for visualization
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

print("Starting Gator vehicle simulation...")

while vis.Run():
    time = system.GetChTime()

    # Render the scene periodically
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update all modules (synchronize)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Update the sensor manager (processes the camera)
    manager.Update()

    # Advance simulation for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    # Advance the system state
    step_number += 1

print("Simulation finished.")