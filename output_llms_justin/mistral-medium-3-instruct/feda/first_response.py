import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.feda as feda
import math

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 0.02

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Point on chassis tracked by the camera (relative to COG)
camera_location = chrono.ChVectorD(0, 0, 1.75)

# Camera to point-of-interest distance
camera_distance = 6.0

# =============================================================================
# Create the simulation system
# =============================================================================

# Create the vehicle system
vehicle = veh.ChWheelVehicle(chrono.GetChronoVehicleDataPath())
vehicle.SetChassisFixed(False)
vehicle.SetVehicleMass(1500)
vehicle.SetInertiaChassis(chrono.ChVectorD(1000, 1500, 2000))
vehicle.SetTireType(veh.ChTire::FEDA)
vehicle.SetTireStepSize(step_size)

# Set contact method
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)

# Initialize the vehicle at the specified location
vehicle.Initialize(chrono.ChCoordinatorSMC(), chrono.ChVectorD(0, 0, 0.5),
                  chrono.ChQuaternionD(1, 0, 0, 0), False)

# Create the vehicle mesh visualization
vehicle.GetChassisBody().GetVisualModel().ClearModels()
vehicle.GetChassisBody().GetVisualModel().AddMeshModel(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"))
vehicle.GetChassisBody().GetVisualModel().SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.5, 0.5, 0.5)))

# Create and initialize the tires
tire_left = feda.ChFEDATire("left_tire")
tire_right = feda.ChFEDATire("right_tire")
tire_left.SetVisualizationType(chrono.ChVisualization::MESH)
tire_right.SetVisualizationType(chrono.ChVisualization::MESH)
tire_left.SetMeshFileName(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_tire.obj"))
tire_right.SetMeshFileName(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_tire.obj"))

# Add tires to the vehicle
vehicle.AddTire(tire_left)
vehicle.AddTire(tire_right)

# =============================================================================
# Create the terrain
# =============================================================================

# Create the rigid terrain
terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.SetContactMaterialProperties(2e6, 0.8)
terrain.SetContactFrictionCoefficient(0.8)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMethod(chrono.ChContactMethod_NSC)

# Create the terrain mesh
terrain_mesh = chrono.ChTriangleMeshConnected()
terrain_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("terrain/flat.obj"), False, True)

# Create the terrain visualization
terrain_visual = chrono.ChVisualModelTriangleMesh()
terrain_visual.SetMesh(terrain_mesh)
terrain_visual.SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.8, 0.8, 0.8)))
terrain_visual.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))

# Add the terrain to the system
terrain.AddVisualModel(terrain_visual)
terrain.AddPatch(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)

# =============================================================================
# Create the driver system
# =============================================================================

# Create the interactive driver
driver = veh.ChInteractiveDriverIRR(vehicle)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)

# =============================================================================
# Create the Irrlicht application
# =============================================================================

# Create the Irrlicht application
app = chronoirr.ChIrrApp(vehicle.GetSystem(), "FEDA Vehicle Simulation", chrono.irr::dimension2d<int>(1280, 720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.irr::core::vector3df(0, 0, 2))
app.SetTimestep(step_size)

# Set the camera to follow the vehicle
app.SetChaseCamera(vehicle.GetChassisBody(), camera_location, camera_distance)

# =============================================================================
# Simulation loop
# =============================================================================

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()

    # Update the driver inputs
    driver.Synchronize(step_size)
    driver.Advance(step_size)

    # Update the vehicle
    vehicle.Synchronize(step_size)
    vehicle.Update(step_size)

    # Advance simulation for one timestep
    app.DoStep()

    app.EndScene()