import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros
import rospy

# ----------------------------
# Simulation Parameters
# ----------------------------
step_size = 1e-3  # 1 millisecond step size
end_time = 10.0   # Total simulation time
chrono.SetChronoDataPath("../../../data/")  # Adjust path as needed

# ----------------------------
# Initialize Chrono System
# ----------------------------
system = chrono.ChSystemSMC()  # Using Smooth Contact Method
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# ----------------------------
# Initialize HMMWV Vehicle
# ----------------------------
vehicle = veh.HMMWV_Full(system)  # Full HMMWV model
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)  # Contact method
vehicle.SetEngineType(veh.HMMWV_EngineSimple())  # Simple engine model
vehicle.SetTireType(veh.HMMWV_TMeasyTire())  # TMeasy tire model
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))  # Initial position and orientation
vehicle.Initialize()

# ----------------------------
# Initialize Terrain
# ----------------------------
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(200, 1, 200), 0.9, 0.01)  # Friction = 0.9, Restitution = 0.01
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 200)  # Optional texture
terrain.Initialize()

# ----------------------------
# Initialize Driver System
# ----------------------------
driver = veh.ChROSInteractiveDriver(vehicle)  # ROS-enabled driver
driver.Initialize()  # Initializes internal state and ROS subscribers

# ----------------------------
# Initialize ROS Manager
# ----------------------------
ros_manager = chros.ChROSPublisher(system, step_size)
ros_manager.Initialize()

# Register ROS handlers
ros_manager.AddClockHandler()  # Publishes simulation time
ros_manager.AddInputHandler(driver)  # Subscribes to driver inputs
ros_manager.AddVehicleStateHandler(vehicle)  # Publishes vehicle state

# ----------------------------
# Simulation Loop
# ----------------------------
time = 0.0
while time < end_time:
    time = system.GetChTime()

    # Synchronize systems
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)

    # Advance simulation
    system.DoStepDynamics(step_size)
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)

    # Update ROS communication
    ros_manager.Update(time, step_size)

# ----------------------------
# Finalize Simulation
# ----------------------------
ros_manager.Finalize()