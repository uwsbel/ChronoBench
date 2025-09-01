import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# Initialize the ChSystem
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the terrain
terrain = veh.RigidTerrain(system)
terrain_file_vis = "Highway_vis.obj"
terrain_file_col = "Highway_col.obj"

terrain_mesh = terrain.AddMesh(chrono.GetChronoDataFile(terrain_file_col),
                               chrono.GetChronoDataFile(terrain_file_vis),
                               chrono.ChVectorD(0, 0, 0))
terrain_mesh.SetContactFrictionCoefficient(0.8)
terrain.Initialize()

# Create the vehicle
vehicle_file = veh.GetDataPath() + "hmmwv/vehicle/HMMWV_Vehicle.json"
tire_file = veh.GetDataPath() + "hmmwv/tire/HMMWV_TMeasyTire.json"
powertrain_file = veh.GetDataPath() + "hmmwv/powertrain/HMMWV_ShaftsPowertrain.json"

vehicle = veh.WheeledVehicle(system, vehicle_file)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create and initialize the tires
for axle in vehicle.GetAxles():
    tireL = veh.TMeasyTire(tire_file)
    vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
    tireR = veh.TMeasyTire(tire_file)
    vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

# Create the powertrain
powertrain = veh.ReadPowertrainJSON(powertrain_file)
vehicle.InitializePowertrain(powertrain)

# Create the driver system
driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)

# Set up Irrlicht visualization
vis = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(), "HMMWV Simulation")
vis.SetHUDLocation(500, 20)
vis.SetSkyBox()
vis.AddTypicalLogo()
vis.AddTypicalLights(irr.vector3df(30, -30, 100), irr.vector3df(30, 50, 100), 250, 130)
vis.AddTypicalCamera(irr.vector3df(6, 3, -6))
vis.AttachVehicle(vehicle.GetVehicle())

# Simulation settings
step_size = 1e-3
realtime_timer = chrono.ChRealtimeStepTimer()

# Simulation loop
while vis.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output HUD
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
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

    # Check if simulation should end
    if time > 1000:  # End after 1000 seconds
        break

# Clean up
vis.GetDevice().drop()