import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os

# Set Chrono data directories
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())

# Simulation parameters
step_size = 2e-3
t_end = 10
render_step_size = 1.0 / 50  # Render at 50 FPS

# HMMWV parameters
initLoc = chrono.ChVectorD(0, 0, 1.0)
initYaw = chrono.ChQuaternionD(1, 0, 0, 0)
powertrain_type = veh.PowertrainModelType_SHAFTS
drive_type = veh.DrivelineTypeWV_AWD
steering_type = veh.SteeringTypeWV_PITMAN_ARM
tire_type = veh.TireModelType_RIGID

# Create the vehicle system
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initYaw))
vehicle.SetPowertrainType(powertrain_type)
vehicle.SetDriveType(drive_type)
vehicle.SetSteeringType(steering_type)
vehicle.SetTireType(tire_type)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

# Apply mesh visualization to all vehicle components
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# Create the terrain with multiple patches
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Flat patch 1: Asphalt
patch1 = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(60, 5, 0.2),
    "concrete"
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 10)

# Flat patch 2: Grass (adjacent to patch 1)
patch2 = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, -7, 0), chrono.QUNIT),
    chrono.ChVectorD(60, 5, 0.2),
    "grass"
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 10)

# Mesh-based patch: Bump
mesh_file = veh.GetDataFile("terrain/meshes/bump.obj")  # Use your own mesh if needed
patch3 = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(20, 0, 0), chrono.QUNIT),
    mesh_file,
    "bump"
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/rock.jpg"), 10, 10)

# Heightmap-based patch: Elevation variation
heightmap_file = veh.GetDataFile("terrain/heightmaps/test64.bmp")  # Use your own heightmap if needed
patch4 = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(40, 0, 0), chrono.QUNIT),
    heightmap_file,
    "heightmap",
    60, 5, 0.5, 0  # size_x, size_y, hMin, hMax
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 60, 5)

terrain.Initialize()

# Create the interactive driver system
driver = veh.ChIrrGuiDriver(
    veh.ChIrrGuiDriver.KeyboardMode.KEYBOARD,
    vehicle.GetChassis()
)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# Create Irrlicht application
app = chronoirr.ChVehicleIrrApp(
    vehicle.GetSystem(),
    "HMMWV on Multi-Patch Terrain",
    chronoirr.dimension2du(1024, 768)
)
app.SetSkyBox()
app.AddTypicalLights()
app.AddTypicalLogo()
app.AddTypicalCamera(chronoirr.vector3df(0, 1.5, 4), chronoirr.vector3df(0, 0, 0))
app.AttachVehicle(vehicle.GetVehicle())

# Simulation loop
render_steps = int(render_step_size / step_size)
step_number = 0
time = 0

while app.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()
    if time >= t_end:
        break

    if step_number % render_steps == 0:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()

    # Get driver inputs
    driver.Synchronize(time)
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    braking = driver.GetBraking()

    # Synchronize subsystems
    vehicle.Synchronize(time, steering, throttle, braking, terrain)
    terrain.Synchronize(time)
    app.Synchronize("HMMWV Simulation", steering, throttle, braking)

    # Advance simulation
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    app.Advance(step_size)

    step_number += 1

print("Simulation complete.")