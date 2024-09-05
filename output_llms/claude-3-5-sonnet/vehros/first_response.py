import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.ros as ros
import math

# Initialize the ChronoROS node
ros_node = ros.ChronoRosNode("chrono_vehicle_node")

# Function to synchronize ROS time with simulation time
def sync_ros_time(sim_time):
    ros_node.update_clock(sim_time)

# ROS callback for driver inputs
def driver_inputs_callback(msg):
    # Update driver inputs based on ROS message
    driver.SetThrottle(msg.throttle)
    driver.SetSteering(msg.steering)
    driver.SetBraking(msg.braking)

# ROS publisher for vehicle state
vehicle_state_pub = ros_node.create_publisher("vehicle_state", "std_msgs/String", 10)

# Initialize the ChSystem
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create the terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))

# Create the vehicle
vehicle = veh.HMMWV_Full(system)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()

# Create the driver system
driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()

# Create the visualization system
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Vehicle Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, -8), chrono.ChVectorD(0, 0, 2))
vis.AddTypicalLights()

# Simulation loop
step_size = 1e-3
sim_time = 0.0

while vis.Run():
    # Synchronize ROS time
    sync_ros_time(sim_time)

    # Process ROS callbacks
    ros_node.spin_some()

    # Update systems
    driver.Synchronize(sim_time)
    vehicle.Synchronize(sim_time, driver.GetInputs(), terrain)
    terrain.Synchronize(sim_time)
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    
    # Advance visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Publish vehicle state
    vehicle_state = vehicle.GetVehicle().GetState()
    vehicle_state_msg = f"Position: {vehicle_state.pos.x}, {vehicle_state.pos.y}, {vehicle_state.pos.z}"
    vehicle_state_pub.publish(vehicle_state_msg)
    
    sim_time += step_size

# Clean up
vis.GetDevice().closeDevice()
system.Clear()