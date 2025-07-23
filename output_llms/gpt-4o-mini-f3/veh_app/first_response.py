import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os

# Set the data path for Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

print(chrono.GetChronoDataPath() + 'vehicle/')

# ----------------------------------------------------------------------------
# Create the Gator vehicle, set parameters, and initialize
# ----------------------------------------------------------------------------
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.5), chrono.ChQuaterniond(1, 0, 0, 0)))
gator.SetEngineType(veh.EngineModelType_SIMPLE)
gator.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
gator.SetDriveType(veh.DrivelineTypeWV_AWD)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# Set collision system type
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ----------------------------------------------------------------------------
# Create the terrain
# ----------------------------------------------------------------------------
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100.0, 100.0)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# ----------------------------------------------------------------------------
# Create the vehicle Irrlicht interface
# ----------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(gator.GetChassis(), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())

# ----------------------------------------------------------------------------
# Create the driver system
# ----------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ----------------------------------------------------------------------------
# Create a sensor manager and add sensors to the manager
# ----------------------------------------------------------------------------
manager = sens.ChSensorManager(gator.GetSystem())

# Create an offset pose for the sensor relative to the vehicle chassis
offset_pose = chrono.ChFramed(chrono.ChVector3d(-8.0, 0, 1.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))

# Create the image buffer for storing camera images
image_buffer = sens.ChImageBufferU8(1280, 720)

# Create a camera sensor and add it to the manager
cam = sens.ChCameraSensor(
    gator.GetChassis(),
    update_rate,
    offset_pose,
    image_buffer,
    90,  # Horizontal FOV
    1280,  # Image width
    720   # Image height
)
cam.SetName("Third Person Camera")
cam.SetLag(lag)
cam.SetCollectionWindow(collection_time)
manager.AddSensor(cam)

# -----------------------------------------------------------------------;
# simulation loop and stream out body data
# POV ray simulation loop
# -----------------------------------------------------------------------;

realtime_step = True
while vis.Run(realtime_step):
    time = gator.GetSystem().GetChTime()

    # Update sensor manager in each step
    manager.Update()

    # --- Get driver inputs ---
    driver_inputs = driver.GetInputs()

    # --- Update the vehicle, terrain, and driver system ---
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)

    vis.Synchronize(time, driver_inputs)
    vis.Render()

    # --- Advance the simulation ---\
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)