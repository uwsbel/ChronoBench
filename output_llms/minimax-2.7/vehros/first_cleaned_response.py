import pychrono as chrono               
import pychrono.vehicle as veh          
import pychrono.ros as chrono_ros       
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist




def init_ros():
    
    rospy.init_node('pychrono_hmmwv_ros', anonymous=True)
    rospy.loginfo("ROS node 'pychrono_hmmwv_ros' started.")




TIME_STEP = 1e-3          
END_TIME  = 20.0          
OUT_DIR   = "./output"    




def make_physical_system():
    
    
    
    system = chrono.ChSystemNSC()

    
    system.Set_G_acc(chrono.ChVectorD(0.0, 0.0, -9.81))

    
    system.SetSolverType(chrono.ChSolver.Type_SOR)
    system.SetMaxIteration(50)

    
    system.SetTimestep(TIME_STEP)

    return system




def create_hmmwv(system):
    

    
    vehicle = veh.HMMWV()

    
    vehicle.SetContactMethod(chrono.ChContactMethod.SMC)

    
    vehicle.SetEngineType(veh.EngineModelType.SIMPLE)

    
    vehicle.SetTransmissionType(veh.TransmissionModelType.SIMPLE)

    
    
    vehicle.SetTireModelType(veh.TireModelType.RIGID)

    
    init_pos = chrono.ChCoordsysD(
        chrono.ChVectorD(0.0, 0.0, 0.8),   
        chrono.ChQuaternionD(1.0, 0.0, 0.0, 0.0)   
    )
    vehicle.SetInitPosition(init_pos)
    vehicle.SetInitVelocity(chrono.ChVectorD(0.0, 0.0, 0.0))

    
    
    vehicle.Initialize(system, True)

    return vehicle




def create_terrain(system):
    

    terrain = veh.RigidTerrain(system)

    
    terrain.SetContactFrictionCoefficient(0.9)      
    terrain.SetContactRestitutionCoefficient(0.0)   
    
    terrain.Initialize()

    return terrain




def create_driver(vehicle):
    

    driver = veh.ChDriver(vehicle.GetVehicle())
    driver.Initialize()

    
    driver.SetThrottle(0.0)   
    driver.SetSteering(0.0)  
    driver.SetBraking(0.0)   

    return driver




def build_ros_bridge(system):
    

    
    
    ros_mgr = chrono_ros.ChROSBridgeManager()
    ros_mgr.SetTimeStep(TIME_STEP)

    
    
    
    clock_pub = rospy.Publisher('/clock', Float64, queue_size=1)

    def on_clock_sync(chrono_time):
        
        clock_pub.publish(Float64(chrono_time))

    ros_mgr.RegisterClockSyncHandler(on_clock_sync)

    
    
    
    
    driver_cmd = {'speed': 0.0, 'steering': 0.0, 'brake': 0.0}

    def driver_input_callback(msg):
        
        driver_cmd['speed']    = max(0.0, msg.linear.x)   
        driver_cmd['steering'] = msg.angular.z            
        driver_cmd['brake']    = max(0.0, -msg.linear.x)  

    driver_sub = rospy.Subscriber(
        '/hmmwv/driver_cmd', Twist, driver_input_callback, queue_size=1
    )

    
    def on_driver_input(chrono_time):
        driver.SetThrottle(driver_cmd['speed'])
        driver.SetSteering(driver_cmd['steering'])
        driver.SetBraking(driver_cmd['brake'])

    ros_mgr.RegisterDriverInputHandler(on_driver_input)

    
    
    
    state_pub = rospy.Publisher('/hmmwv/vehicle_state', Twist, queue_size=1)

    def on_vehicle_state(chrono_time):
        
        state = vehicle.GetVehicle().GetState()

        twist_msg = Twist()
        twist_msg.linear.x  = state.v.x
        twist_msg.linear.y  = state.v.y
        twist_msg.linear.z  = state.v.z
        twist_msg.angular.x = state.w.x
        twist_msg.angular.y = state.w.y
        twist_msg.angular.z = state.w.z
        state_pub.publish(twist_msg)

    ros_mgr.RegisterVehicleStateHandler(on_vehicle_state)

    return ros_mgr




def simulation_loop(system, vehicle, terrain, driver, ros_mgr):
    

    rospy.loginfo("Starting simulation loop …")

    
    system.SetTimestep(TIME_STEP)

    
    while not rospy.is_shutdown():
        
        if system.GetChTime() >= END_TIME:
            rospy.loginfo("Simulation finished (end time reached).")
            break

        
        
        
        ros_mgr.Synchronize(system.GetChTime())

        
        
        
        system.DoStepDynamics(TIME_STEP)

        
        
        
        vehicle.Advance(TIME_STEP)   
        terrain.Advance(TIME_STEP)   
        driver.Advance(TIME_STEP)    

        
        
        
        ros_mgr.Update()

    rospy.loginfo("Simulation loop terminated.")




if __name__ == '__main__':
    try:
        
        init_ros()

        
        system = make_physical_system()

        
        vehicle = create_hmmwv(system)

        
        terrain = create_terrain(system)

        
        driver = create_driver(vehicle)

        
        ros_mgr = build_ros_bridge(system)

        
        simulation_loop(system, vehicle, terrain, driver, ros_mgr)

    except rospy.ROSInterruptException:
        
        pass