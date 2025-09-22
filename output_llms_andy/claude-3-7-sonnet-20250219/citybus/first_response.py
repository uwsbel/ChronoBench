import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import os

# ---------------------------------------------------------------------
# Initialize the PyChrono environment and core components

print("PyChrono CityBus Simulation")

# Set output directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
output_dir = "CityBus_output/"
if not os.path.exists(output_dir):
    os.mkdir(output_dir)

# Create the simulation system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ---------------------------------------------------------------------
# Create the terrain

terrain = veh.RigidTerrain(system)
patch_material = chrono.ChMaterialSurfaceSMC()
patch_material.SetFriction(0.9)
patch_material.SetRestitution(0.01)

# Create a flat terrain patch
terrain_mesh = chrono.ChTriangleMeshConnected()
terrain_mesh.LoadWavefrontMesh(chrono.GetChronoDataPath() + "vehicle/terrain/meshes/flat_terrain.obj", False, True)
terrain_mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
terrain_patch = terrain.AddPatch(patch_material, terrain_mesh, 0.01)

# Set visualization properties for the terrain
terrain_patch.SetTexture(chrono.GetChronoDataPath() + "vehicle/terrain/textures/tile4.jpg", 200, 200)
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the vehicle

# Set up city bus JSON file paths
vehicle_file = chrono.GetChronoDataPath() + "vehicle/citybus/vehicle/CityBus_Vehicle.json"
powertrain_file = chrono.GetChronoDataPath() + "vehicle/citybus/powertrain/CityBus_SimpleMapPowertrain.json"
tire_file = chrono.GetChronoDataPath() + "vehicle/citybus/tire/CityBus_TMeasyTire.json"

# Create the CityBus vehicle
citybus = veh.WheeledVehicle(system, vehicle_file)
citybus.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
citybus.SetChassisVisualizationType(veh.VisualizationType_MESH)
citybus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
citybus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
citybus.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create and initialize the powertrain
powertrain = veh.ReadPowertrainJSON(powertrain_file)
citybus.InitializePowertrain(powertrain)

# Create and initialize the tires
for axle in range(citybus.GetNumberAxles()):
    for side in range(2):
        wheel = citybus.GetWheel(axle, side)
        tire = veh.ReadTireJSON(tire_file)
        citybus.InitializeTire(tire, wheel, veh.VisualizationType_MESH)

# ---------------------------------------------------------------------
# Create the driver system

# Create the interactive driver
driver = veh.ChIrrGuiDriver(citybus.GetVehicle())
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.2)
driver.SetBrakingDelta(0.5)
driver.Initialize()

# ---------------------------------------------------------------------
# Set up the Irrlicht visualization

# Create the Irrlicht application
vis = veh.ChWheeledVehicleIrrApp(citybus.GetVehicle(), "CityBus Simulation", irr.dimension2du(1280, 720))
vis.AddTypicalLights()
vis.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-10, 5, 0), chrono.ChVectorD(0, 0, 0))
vis.SetChaseCamera(chrono.ChVectorD(5, 2, 0), 6.0, 0.5)
vis.SetTimestep(5e-3)
vis.AttachVehicle(citybus.GetVehicle())
vis.Initialize()

# ---------------------------------------------------------------------
# Simulation settings

step_size = 5e-3  # step size in seconds
realtime_timer = chrono.ChRealtimeStepTimer()
render_step_size = 1.0 / 50  # Render at 50 FPS
render_timer = 0.0

# ---------------------------------------------------------------------
# Simulation loop

while vis.GetDevice().run():
    # Render scene
    if system.GetChTime() >= render_timer:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_timer += render_step_size
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update modules (process inputs, update states, etc.)
    terrain.Synchronize(system.GetChTime())
    citybus.Synchronize(system.GetChTime(), driver_inputs, terrain)
    vis.Synchronize(driver_inputs, system.GetChTime())
    
    # Advance simulation for one timestep
    system.DoStepDynamics(step_size)
    
    # Print vehicle speed
    speed_kmh = citybus.GetVehicle().GetVehicleSpeed() * 3.6
    print(f"Time: {system.GetChTime():.3f}s, Speed: {speed_kmh:.1f} km/h")
    
    # Try to maintain real-time simulation
    realtime_timer.Spin(step_size)

# Clean up
del vis
del citybus
del terrain