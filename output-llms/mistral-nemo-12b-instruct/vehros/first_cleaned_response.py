import chrono as c
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


c.ChSystem.SetChronoDataPath("path_to_chrono_data")
my_system = c.ChSystem_SI()
my_system.Set_Gravity(c.ChVectorD(0, 0, -9.81))


terrain = c.ChTerrain()
terrain.SetMaterial(c.ChMaterialSurfaceSMC(0.9, 0.0, 0.0))  
terrain.AddBox(100, 100, 0.1, c.ChVectorD(0, 0, -0.1))  
my_system.Add(terrain)


hmmwv = c.ChBodyEasyBox(0.5, 2, 1.8, 1000, True)  
hmmwv.SetPos(c.ChVectorD(0, 0, 0.9))  
my_system.Add(hmmwv)


engine = c.ChTruckEngine()
tire = c.ChTire()
hmmwv.AddEngine(engine)
hmmwv.AddTire(tire)


engine.SetPower(100)  
tire.SetContactMethod(c.CONTACT_METHOD_PATCH)  
tire.SetTireModel(c.TIRE_MODEL_FLEXIBLE)  


rospy.init_node('hmmwv_simulation', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)


driver = c.ChTruckDriver()
driver.SetSteeringInput(0)  
driver.SetThrottleInput(0)  


rate = rospy.Rate(10)  
while not rospy.is_shutdown():
    
    my_system.DoStepDynamics(0.01)  

    
    twist = Twist()
    twist.linear.x = driver.GetThrottleInput()
    twist.angular.z = driver.GetSteeringInput()
    pub.publish(twist)

    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.pose.pose.position.x = hmmwv.GetPos()[0]
    odom.pose.pose.position.y = hmmwv.GetPos()[1]
    odom_pub.publish(odom)

    rate.sleep()