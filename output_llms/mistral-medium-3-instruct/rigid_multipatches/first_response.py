import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np
import os

# =============================================================================
# Initialize PyChrono and create the simulation system
# =============================================================================

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), '../../data/'))

# Create the Chrono simulation system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravitational acceleration

# =============================================================================
# Create the terrain system
# =============================================================================

# Create the terrain
terrain = veh.RigidTerrain(system)

# Add a large flat patch (asphalt)
asphalt_patch = terrain.AddPatch(chrono.ChCoordinatesysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                                20, 20,  # Dimensions (width, length)
                                0.1,     # Thickness
                                True,    # Collision enabled
                                0.8,     # Friction coefficient
                                1e6,     # Normal stiffness
                                1e5,     # Normal damping
                                1e6,     # Tangential stiffness
                                1e5)     # Tangential damping
asphalt_patch.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
asphalt_patch.SetTexture(veh.GetDataFile("terrain/textures/asphalt.jpg"))

# Add a flat patch with different texture (concrete)
concrete_patch = terrain.AddPatch(chrono.ChCoordinatesysD(chrono.ChVectorD(15, 0, 0), chrono.QUNIT),
                                20, 20,
                                0.1,
                                True,
                                0.9,
                                1e6,
                                1e5,
                                1e6,
                                1e5)
concrete_patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
concrete_patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"))

# Add a mesh-based patch for a bump
bump_patch = terrain.AddPatch(chrono.ChCoordinatesysD(chrono.ChVectorD(5, 0, 0), chrono.QUNIT),
                             veh.GetDataFile("terrain/meshes/bump.obj"),
                             True,
                             0.7,
                             1e6,
                             1e5,
                             1e6,
                             1e5)
bump_patch.SetColor(chrono.ChColor(0.8, 0.6, 0.4))

# Add a heightmap-based patch for varying elevations
heightmap_patch = terrain.AddPatch(chrono.ChCoordinatesysD(chrono.ChVectorD(-5, 0, 0), chrono.QUNIT),
                                  veh.GetDataFile("terrain/height_maps/test64.bmp"),
                                  10, 10,  # Dimensions
                                  0.1,     # Height scale
                                  True,
                                  0.6,
                                  1e6,
                                  1e5,
                                  1e6,
                                  1e5)
heightmap_patch.SetColor(chrono.ChColor(0.4, 0.8, 0.4))
heightmap_patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"))

# Finalize the terrain
terrain.Initialize()

# =============================================================================
# Create the HMMWV vehicle
# =============================================================================

# Create the vehicle system
vehicle = veh.WheeledVehicle(chrono.GetVehicleDataFile("hmmwv/vehicle/HMMWV_Vehicle.json"))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetInitializePosition(chrono.ChVectorD(0, 0.5, 0))
vehicle.SetInitializeRotation(chrono.ChQuaternionD(1, 0, 0, 0))

# Set vehicle parameters
vehicle.SetChassisFixed(False)
vehicle.SetEngineType(veh.EngineModelType::SHAFT)
vehicle.SetDrivelineType(veh.DrivelineType::AWD)
vehicle.SetTireType(veh.TireModelType::RIGID)

# Initialize the vehicle
vehicle.Initialize(chrono.GetChronoDataPath() + "/vehicle/hmmwv/")

# Create the vehicle system and add it to the simulation system
vehicle_system = veh.ChVehicleSystem(system)
vehicle_system.AddVehicle(vehicle)

# =============================================================================
# Create the driver system
# =============================================================================

# Create an interactive driver
driver = veh.ChInteractiveDriverIRR()

# Initialize the driver
driver.Initialize()

# =============================================================================
# Create the visualization system
# =============================================================================

# Create the Irrlicht application
app = chronoirr.ChIrrApp(vehicle_system, "HMMWV Simulation", chrono.ChVectorD(1280, 720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(5, 2, 0))
app.SetTimestep(0.01)  # Set the timestep

# Customize the camera
camera = app.GetCamera()
camera.SetPosition(chrono.ChVectorD(5, 2, 0))
camera.SetAimPoint(chrono.ChVectorD(0, 0, 0))

# =============================================================================
# Simulation loop
# =============================================================================

# Number of simulation steps
num_steps = 1000

# Simulation loop
while app.GetDevice().run():
    # Begin the scene
    app.BeginScene()

    # Render the scene
    app.DrawAll()

    # Advance the simulation for one timestep for all modules
    driver.Synchronize(0.01)
    vehicle_system.Synchronize(0.01, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    app.DoStep()

    # End the scene
    app.EndScene()