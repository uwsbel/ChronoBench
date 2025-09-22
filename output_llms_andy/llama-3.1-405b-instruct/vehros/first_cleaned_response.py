import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import rospy
from ros_chrono.msg import VehicleState, DriverInputs


chrono.SetChronoDataPath('/usr/local/share/chrono/data/')
chrono.ChCollisionSystem.SetDefault(chrono.ChCollisionSystem.BULLET)


sys = chrono.ChSystemNSC()


vehicle = veh.HMMWV_Reduced(sys, 
                            contact_method=chrono.ChMaterialSurface.NSC, 
                            engine_type=veh.ChPart_Engine.SIMPLE, 
                            tire_model=veh.ChPart_Tire.RIGID)


vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 0.5))
vehicle.SetInitFwdVel(5)
vehicle.SetSteering(0)
vehicle.SetThrottle(0)
vehicle.SetBraking(0)


terrain = veh.RigidTerrain(sys)
terrain.SetFriction(0.9)
terrain.SetRestitution(0.01)
sys.Add(terrain)


driver = veh.ChIrrNodeApplicationHMMWV(vehicle)
sys.Add(driver)


rospy.init_node('pychrono_simulation')


def clock_sync_handler(msg):
    sys.SetChTime(msg.clock)

def driver_inputs_handler(msg):
    vehicle.SetSteering(msg.steering)
    vehicle.SetThrottle(msg.throttle)
    vehicle.SetBraking(msg.braking)

def vehicle_state_publisher():
    pub = rospy.Publisher('vehicle_state', VehicleState, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        state = VehicleState()
        state.position = vehicle.GetVehiclePos()
        state.velocity = vehicle.GetVehicleFwdVel()
        pub.publish(state)
        rate.sleep()


rospy.Subscriber('clock', Clock, clock_sync_handler)
rospy.Subscriber('driver_inputs', DriverInputs, driver_inputs_handler)


ros_manager = sens.ChRosManager(sys)
ros_manager.AddPublisher(vehicle_state_publisher)


while not rospy.is_shutdown():
    
    sys.DoStepDynamics(0.01)
    terrain.Synchronize()
    driver.Synchronize()
    
    
    sys.DoStepDynamics(0.01)
    terrain.Advance(0.01)
    driver.Advance(0.01)
    
    
    ros_manager.Update()