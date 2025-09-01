import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.ros as ros
import math


ros_node = ros.ChronoRosNode("chrono_vehicle_node")


def sync_ros_time(sim_time):
    ros_node.update_clock(sim_time)


def driver_inputs_callback(msg):
    
    driver.SetThrottle(msg.throttle)
    driver.SetSteering(msg.steering)
    driver.SetBraking(msg.braking)


vehicle_state_pub = ros_node.create_publisher("vehicle_state", "std_msgs/String", 10)


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))


vehicle = veh.HMMWV_Full(system)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Vehicle Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, -8), chrono.ChVectorD(0, 0, 2))
vis.AddTypicalLights()


step_size = 1e-3
sim_time = 0.0

while vis.Run():
    
    sync_ros_time(sim_time)

    
    ros_node.spin_some()

    
    driver.Synchronize(sim_time)
    vehicle.Synchronize(sim_time, driver.GetInputs(), terrain)
    terrain.Synchronize(sim_time)
    
    
    system.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    vehicle_state = vehicle.GetVehicle().GetState()
    vehicle_state_msg = f"Position: {vehicle_state.pos.x}, {vehicle_state.pos.y}, {vehicle_state.pos.z}"
    vehicle_state_pub.publish(vehicle_state_msg)
    
    sim_time += step_size


vis.GetDevice().closeDevice()
system.Clear()