import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh

# --- Create the simulation system ---
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# --- Create the terrain ---
terrain = chrono.ChTerrain()
terrain.Create(system, "terrain", chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(100, 2, 100), 20, 20)
terrain.SetFriction(0.8)
terrain.SetRestitution(0.1)
system.Add(terrain)

# --- Create the M113 vehicle ---
vehicle = veh.ChM113()
vehicle.SetBodyFixed(False)
vehicle.SetPos(chrono.ChVectorD(0, 1, 0))
vehicle.SetYaw(chrono.ChVectorD(0, 1, 0))
system.Add(vehicle)

# --- Create the driver system ---
driver = veh.ChDriver()
driver.SetSteeringDelta(20)  # degrees
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)
driver.SetVehicle(vehicle)
driver.SetMode(veh.ChDriver.ACCELERATE_BRAKE)

# --- Set initial vehicle speed ---
vehicle.SetForwardVelocity(0)
vehicle.SetSteering(0)
vehicle.SetThrottle(0)
vehicle.SetBraking(0)

# --- Create the Irrlicht application for visualization ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('M113 Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddTypicalLights()

# --- Simulation loop ---
timestep = 0.005
time = 0.0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # --- Update the driver ---
    driver.Synchronize(timestep)

    # --- Update the vehicle ---
    vehicle.Synchronize(timestep)

    # --- Update the system ---
    system.DoStepDynamics(timestep)

    time += timestep
    #print(f"Time: {time:.2f}") # Optional: print the simulation time

vis.GetIrrlichtApplication().Close()