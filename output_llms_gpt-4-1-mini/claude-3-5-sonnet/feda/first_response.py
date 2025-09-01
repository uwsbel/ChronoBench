import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# Initialize the ChronoEngine and vehicle components
chrono.SetChronoDataPath("C:/Program Files/Chrono/data/")
veh.SetDataPath("C:/Program Files/Chrono/data/vehicle/")

# Create the simulation system
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         200, 200)

# Set terrain visualization properties
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle
vehicle = veh.WheeledVehicle(system, veh.GetDataFile("generic/vehicle/Vehicle_DoubleWishbones.json"))
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))

# Set the visualization type for vehicle parts
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create and initialize the powertrain
powertrain = veh.SimplePowertrain(veh.GetDataFile("generic/powertrain/SimplePowertrain.json"))
vehicle.InitializePowertrain(powertrain)

# Create the tires
tire_data = veh.TMeasyTireData(veh.GetDataFile("generic/tire/TMeasyTire.json"))
for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        tire = veh.TMeasyTire(tire_data)
        vehicle.InitializeTire(tire, wheel, veh.VisualizationType_MESH)

# Create the driver system
driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(), "FEDA Vehicle Simulation")

vis.SetHUDLocation(500, 20)
vis.SetSkyBox()
vis.AddTypicalLights()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AttachVehicle(vehicle.GetVehicle())

# Set the camera to follow the vehicle
vis.SetChaseCameraPosition(chrono.ChVectorD(-8, 3, -5))
vis.SetChaseCameraMultipliers(1.0, 5.0, 0.6)

# Simulation settings
step_size = 1e-3
irr_fps = 50

realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Ensure real-time simulation
    realtime_timer.Spin(step_size)

    # Render scene at specified FPS
    if vis.GetDevice().run():
        if vis.GetDevice().getTimer().getTime() - vis.GetLastRenderTime() >= 1000.0 / irr_fps:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.UpdateHUD()
            vis.GetLastRenderTime() = vis.GetDevice().getTimer().getTime()

vis.GetDevice().closeDevice()