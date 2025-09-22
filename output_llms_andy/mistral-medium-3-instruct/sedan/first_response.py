import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.wheeled_vehicle as wheeled_vehicle
import pychrono.vehicle.terrain as terrain
import pychrono.vehicle.driver as driver
import pychrono.vehicle.tmeasy as tmeasy

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 1e-3

# Simulation end time
tend = 100

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.0)

# =============================================================================
# Create the systems and simulation
# =============================================================================

# Create the vehicle system
vehicle = veh.ChWheeledVehicle(veh.GetDataFile("vehicle/bmw_e90/vehicle.json"))

# Set visualization type
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create the ground
terrain = terrain.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9f)
patch_mat.SetRestitution(0.1f)
patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# Create the vehicle TMeasy tire force elements
tire = tmeasy.TMeasyTire("TMeasy")
tire.SetVisualizationType(veh.VisualizationType_MESH)
vehicle.InitializeTires(tire)

# Initialize the vehicle at the specified location
vehicle.SetChassisFixed(False)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT),
                  chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -1)))

# Create the interactive driver system
driver = driver.ChInteractiveDriverIRR(vehicle)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)
driver.Initialize()

# =============================================================================
# Create the Irrlicht application
# =============================================================================

# Create the Irrlicht visualization
app = chronoirr.ChIrrApp(vehicle.GetSystem(), "BMW E90 Sedan Simulation", chronoirr.dimension2du(1280, 720))

# Set the camera
app.SetChaseCamera(trackPoint, 6.0, 0.5)

# Set up the light
light = chronoirr.ChIrrLightDirectional(app.GetDevice(), chrono.ChVectorD(30, 30, 100))
light.SetLightColor(chrono.ChColor(1.0f, 1.0f, 1.0f))

# Set the skybox
app.SetSkyBox(chrono.GetChronoDataFile("skybox/").c_str())

# =============================================================================
# Simulation loop
# =============================================================================

# Number of simulation steps between two 3D view render frames
step_size2 = step_size
render_steps = int(1 / (step_size2 * 50))

app.SetTimestep(step_size2)
app.AssetBindAll()
app.AssetUpdateAll()

# Simulation loop
while app.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()

    # Exit simulation
    if time > tend:
        break

    # Update driver inputs
    driver.Synchronize(time)
    driver.Advance(step_size)

    # Update modules (process inputs from other modules)
    driver.Advance(step_size)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    vehicle.Advance(step_size)

    # Update visualization
    if (app.GetStepNumber() % render_steps == 0):
        app.AssetUpdateAll()