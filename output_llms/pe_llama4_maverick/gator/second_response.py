import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Correctly set the Chrono data path
chrono.SetChronoDataPath('/path/to/chrono/data')  # Replace with actual path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.0)  # Adjusted to be above the terrain
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the Gator vehicle, set parameters, and initialize
vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Visualization settings
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain with multiple patches
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Define patch materials based on contact method
if contact_method == chrono.ChContactMethod_NSC:
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
else:
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)

# Add patches
patch1 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(-50, 0, 0), chrono.QUNIT), 50, 50)
patch1.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 50, 50)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

patch2 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 50, 50)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 50, 50)
patch2.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

patch3 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(50, 0, 0), chrono.QUNIT), 50, 50)
patch3.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 50, 50)
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))

# Create a patch with a height map
mesh_file = chrono.GetChronoDataFile("terrain/height_maps/test64.bmp")
patch4 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(-50, -50, 0), chrono.QUNIT), mesh_file, 128, 128, 0, 4, True)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 128, 128)
patch4.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

terrain.Initialize()

# Add a bump to one of the patches (example for patch2)
# Here, we simply move the center of patch2 up by 1 unit to simulate a bump
# patch2.SetLocation(chrono.ChVector3d(0, 0, 1))

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)