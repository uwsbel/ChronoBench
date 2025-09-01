import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# ==============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ---------------
# Simulation setup
# ---------------

# Create the MAN 10t truck vehicle with TMeasy tires
initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)
vehicle = veh.MAN_10t(initLoc, initRot)

# Set the tire model to TMeasy
vehicle.SetTireType(veh.TireModelType_TMEASY)

# Optionally, enable/disable collision for the vehicle chassis and/or tires
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetTireCollisionType(veh.CollisionType_ENVELOPE)

# Optionally, enable visualization of the vehicle components
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# Create the terrain
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                         100.0, 100.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# Create the interactive driver system
driver = veh.ChIrrGuiDriver(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(driver.GetSteeringMax() / steering_time * 0.02)
driver.SetThrottleDelta(driver.GetThrottleMax() / throttle_time * 0.02)
driver.SetBrakingDelta(driver.GetBrakingMax() / braking_time * 0.02)

# Simulation loop
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Draw scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(0.02)
    terrain.Advance(0.02)
    vehicle.Advance(0.02)
    vis.Advance(0.02)

    # Increment time
    vehicle.GetSystem().DoStepDynamics(0.02)

# Cleanup
del vis
del driver
del vehicle
del terrain