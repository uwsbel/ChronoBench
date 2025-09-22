import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irrl
import pychrono.sensor as sens
import pychrono.ros as ros
import math

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 1e-3

# Simulation end time
t_end = 30

# =============================================================================
# Create the simulation system and add items
# =============================================================================

# Create the Chrono physical system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Set solver parameters
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetMaxItersSolverSpeed(150)
sys.SetMaxItersSolverStab(150)
sys.SetTolForce(1e-5)

# Create the vehicle system
vehicle = veh.WheelVehicle(sys)
vehicle.SetChassisFixed(False)
vehicle.SetInitializePosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))

# =============================================================================
# Create the terrain
# =============================================================================

# Create a rigid terrain
terrain = veh.RigidTerrain(sys)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)

# Create a flat patch
patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(20, 20, 0),
                         chrono.ChVectorD(0, 0, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# =============================================================================
# Create the HMMWV vehicle
# =============================================================================

# Create the HMMWV vehicle
hmmwv = veh.HMMWV(sys)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetEngineType(veh.ChEngineModelType::SHARED_SOFTWARE)
hmmwv.SetTireModelType(veh.ChTireModelType::TMEASY)
hmmwv.SetDriveType(veh.ChVehicleDriveType::REAR)

# Initialize the vehicle at the specified position
hmmwv.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))

# =============================================================================
# Create the driver system
# =============================================================================

# Create a simple driver system
driver = veh.ChPathFollowerDriver(hmmwv, "path", 3.0, 0.75)
driver.SetPath(veh.ChBezierCurve.CreateSimplePath(chrono.ChVectorD(0, 0, 0.5),
                                                 chrono.ChVectorD(10, 0, 0.5),
                                                 chrono.ChVectorD(10, 10, 0.5),
                                                 chrono.ChVectorD(0, 10, 0.5)))

# =============================================================================
# Create the visualization system
# =============================================================================

# Create the Irrlicht application
app = irrl.ChIrrApp(sys, "HMMWV Simulation", irrl.dimension2d(1280, 720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(irrl.core.vector3df(0, 5, 2), irrl.core.vector3df(0, 0, 0.5))
app.AssetBindAll()
app.AssetUpdateAll()

# =============================================================================
# Create the ROS interface
# =============================================================================

# Initialize ROS node
ros_node = ros.ChRosNode("hmmwv_simulation")

# Create ROS manager
ros_manager = ros.ChRosManager(sys, ros_node, step_size)

# Register ROS handlers
ros_manager.RegisterHandler(ros.ChRosClockHandler())
ros_manager.RegisterHandler(ros.ChRosDriverInputHandler(driver))
ros_manager.RegisterHandler(ros.ChRosVehicleStateHandler(hmmwv))

# =============================================================================
# Simulation loop
# =============================================================================

# Number of simulation steps
num_steps = int(math.ceil(t_end / step_size))

# Simulation loop
for i in range(num_steps):
    # Synchronize systems
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time)

    # Advance simulation for one timestep
    sys.DoStepDynamics(step_size)

    # Update ROS manager
    ros_manager.Update()

    # Update visualization
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()

    # Update simulation time
    time += step_size

    # Exit if ESC key is pressed
    if app.GetDevice().isWindowActive() and app.GetDevice().getEventReceiver().IsKeyDown(irrl.KEY_ESCAPE):
        break