import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.wheeled_vehicle as wheeled_vehicle
import pychrono.vehicle.terrain as terrain

# =============================================================================
# Initialize the PyChrono environment and core components
# =============================================================================

# Create the Chrono::Engine physical system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# =============================================================================
# Set up visualization using Irrlicht
# =============================================================================

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('M113 Vehicle Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(10, 5, 0))
vis.AddTypicalLights()
vis.SetCameraVertical(chrono.CameraVerticalDir::Z)
vis.SetCameraAngle(chrono.ChVectorD(0.5 * chrono.CH_PI, 0, 0))
vis.SetCameraTracking(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))

# =============================================================================
# Create the terrain
# =============================================================================

# Create a rigid terrain
terrain = veh.RigidTerrain(sys)
terrain.SetContactFriction(0.8)  # Friction coefficient
terrain.SetContactRestitution(0.1)  # Restitution coefficient
terrain.SetContactMaterialProperties(2e7, 0.3)  # Young's modulus, Poisson ratio

# Create the terrain surface
patch_size = 100
patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(patch_size, 0, 0),
                         chrono.ChVectorD(0, 0, patch_size), "flat_patch")
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 20)

# =============================================================================
# Create the M113 vehicle
# =============================================================================

# Create the vehicle system
vehicle = veh.WheeledVehicle(sys, veh.VehicleSide::DOUBLE_SIDE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetInitFwdVel(10)  # Initial forward velocity (m/s)

# Create and initialize the M113 vehicle model
m113 = veh.M113()
m113.Initialize(vehicle)

# =============================================================================
# Create the driver system
# =============================================================================

# Create a driver system
driver = veh.ChDriver()
driver.Initialize(vehicle)

# Set up a simple path-following driver
path = veh.ChPath()
path.AddPoint(0, 0, 0)
path.AddPoint(50, 0, 0)
path.AddPoint(100, 0, 0)
driver.SetPath(path)

# Set driver parameters
driver.SetSteeringControllerType(veh.ChSteeringController::PATH_FOLLOWING)
driver.SetThrottleControllerType(veh.ChThrottleController::SPEED)
driver.SetDesiredSpeed(10)  # m/s

# =============================================================================
# Finalize vehicle and simulation setup
# =============================================================================

# Finalize vehicle construction
vehicle.Initialize()

# Set up the visual assets for the vehicle
veh.SetChassisVisualizationType(vehicle, veh.VisualizationType::MESH)
veh.SetSuspensionVisualizationType(vehicle, veh.VisualizationType::PRIMITIVES)
veh.SetSteeringVisualizationType(vehicle, veh.VisualizationType::PRIMITIVES)
veh.SetWheelVisualizationType(vehicle, veh.VisualizationType::MESH)

# =============================================================================
# Simulation loop
# =============================================================================

# Simulation parameters
time_step = 0.01  # 100 Hz
sim_time = 10.0  # Simulation duration (seconds)

# Perform the simulation loop
while vis.Run():
    time = sys.GetChTime()

    # Exit simulation after specified duration
    if time > sim_time:
        break

    # Synchronize systems
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, veh.WheeledVehicle::DriverInputs(driver.GetSteering(), driver.GetThrottle(), driver.GetBraking()))

    # Advance simulation by one step
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    # Print vehicle information periodically
    if int(time * 10) % 10 == 0:
        pos = vehicle.GetChassis().GetPos()
        vel = vehicle.GetChassis().GetPos_dt()
        print(f"Time: {time:.2f} s | Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) m | Speed: {vel.Length():.2f} m/s")