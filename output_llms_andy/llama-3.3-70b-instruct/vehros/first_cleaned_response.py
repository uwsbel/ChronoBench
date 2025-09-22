import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import rospy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


sys = chrono.ChSystemNSC()


vehicle = veh.HMMWV()
vehicle.AddDrivers()
vehicle.AddTires([veh.TireType.TIRE_RIGID])
vehicle.SetContactMethod(chrono.ChContactMethod.NSC)
vehicle.SetEngineType(veh.EngineType.SINGLE_GEAR)
vehicle.SetTireModel(veh.TireModel.RIGID)


terrain = veh.RigidTerrain()
terrain.SetFriction(0.8)
terrain.SetRestitution(0.2)


sys.Add(vehicle)
sys.Add(terrain)


driver = veh.ChDriversCallback()
vehicle.AddDriver(driver)


rospy.init_node('pychrono_simulation')
clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
driver_input_pub = rospy.Publisher('/driver_input', Twist, queue_size=10)
vehicle_state_pub = rospy.Publisher('/vehicle_state', Twist, queue_size=10)


def simulation_loop():
    time_step = 0.01
    while not rospy.is_shutdown():
        
        sys.DoStepDynamics(time_step)

        
        driver.Synchronize(time_step)

        
        clock = Clock()
        clock.clock = rospy.Time.now()
        clock_pub.publish(clock)

        driver_input = Twist()
        driver_input.linear.x = driver.GetThrottle()
        driver_input.angular.z = driver.GetSteering()
        driver_input_pub.publish(driver_input)

        vehicle_state = Twist()
        vehicle_state.linear.x = vehicle.GetSpeed()
        vehicle_state.angular.z = vehicle.GetOrientation()
        vehicle_state_pub.publish(vehicle_state)

        
        sys.DoStepDynamics(time_step)


simulation_loop()