import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh

# --- Create the simulation system ---
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# --- Create the terrain ---
terrain = chrono.ChTerrain()
terrain.Create(system, "terrain", chrono.ChVectorD(0, -1, 0), chrono.ChVectorD(100, 0, 100), 25, 25)
terrain.SetFriction(0.8)
terrain.SetRestitution(0.1)
system.Add(terrain)

# --- Create the M113 vehicle ---
vehicle = veh.ChM113()
vehicle.SetVehicleModelPath("data/M113/")  # Ensure this path is correct
vehicle.Initialize(system)
vehicle.SetChassisHeight(0.7)
vehicle.SetEngineType(veh.ChM113.EngineType.GASOLINE)
vehicle.SetTrackOffset(0.0)
vehicle.SetTrackWidth(1.5)

# Set initial position and orientation
vehicle.SetPos(chrono.ChVectorD(0, 1, 0))
vehicle.SetTransform(chrono.ChFrameD(chrono.ChVectorD(0, 1, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))


# --- Create the driver system ---
driver = veh.ChDriver()
driver.SetInput(veh.ChDriver.INPUT_TYPE.SPEED, 1.0)  # Set speed
driver.SetInput(veh.ChDriver.INPUT_TYPE.STEERING, 0.0)  # Set steering
driver.Initialize(system, vehicle)

# --- Create the Irrlicht visualization ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('M113 Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5))
vis.AddTypicalLights()

# --- Simulation loop ---
timestep = 0.005
time = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(timestep)
    driver.Synchronize(timestep)

    time += timestep

    if time > 10:
        driver.SetInput(veh.ChDriver.INPUT_TYPE.SPEED, 0.0)  # Stop after 10 seconds
        
    # Update the camera to follow the vehicle
    camera_pos = vehicle.GetPos() + chrono.ChVectorD(0, 2, -5)
    vis.GetCamera(0).SetPos(camera_pos)
    vis.GetCamera(0).SetLookAt(vehicle.GetPos())