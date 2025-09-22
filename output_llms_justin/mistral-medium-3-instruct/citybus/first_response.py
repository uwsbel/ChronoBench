import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.citybus as citybus

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 0.02  # 50 FPS

# Simulation end time
tend = 100.0

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.0)

# =============================================================================
# Create the systems and set solver parameters
# =============================================================================

# Create the vehicle system
vehicle = citybus.CityBus()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType::TMEASY)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

vehicle.GetSystem().Set_G_acc(chrono.ChVectorD(0, 0, -9.81,))

# =============================================================================
# Create the terrain
# =============================================================================

# Create the rigid terrain
terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitution(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain.SetPlotType(veh.Terrain::PLOT_TEXTURE, True)

# Load a custom texture for the terrain
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/dirt.jpg"), 200, 200)
terrain.Initialize(0, 0, 0, 0)

# =============================================================================
# Create the driver system
# =============================================================================

# Create the interactive driver
driver = veh.ChInteractiveDriverIRR(vehicle)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.04)
driver.Initialize()

# =============================================================================
# Create the visualization system
# =============================================================================

# Create the Irrlicht application
app = chronoirr.ChIrrApp(vehicle.GetSystem(), "CityBus Demo", chrono.ChVectorD(1280, 720))

# Set up the camera
app.SetTimestep(step_size)
app.SetTryRealtime(True)
app.SetCameraVertical(chrono.ChCameraVertical_DIRTYZ)
app.SetChaseCamera(trackPoint, 6.0, 0.5)
app.SetSkyBox()

# Customize vehicle visualization
app.AssetBindAll()
app.AssetUpdateAll()

# =============================================================================
# Simulation loop
# =============================================================================

# Number of simulation steps between two 3D view render frames
step_size2 = step_size
render_steps = int(step_size2 / step_size)

# Initialize simulation frame counter
step_number = 0

while app.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    # End simulation
    if time >= tend:
        break

    # Render scene
    app.BeginScene()
    app.DrawAll()
    app.DoStepDynamics(step_size)
    app.EndScene()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, step_size, driver.GetInputs())

    # Update visualization
    if step_number % render_steps == 0:
        app.GetSystem().DoStepDynamics(step_size)
        app.DoStepDynamics(step_size)

    step_number += 1

print("Simulation completed")