importchrono
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


chrono.SetChronoDataPath()
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


hmmwv = chrono.ChHMMWV()
hmmwv.SetContactMethod(chrono.CHRONO>Contact method)  
hmmwv.SetEngineType(chrono.ChHMMWVEngineType_SIMPLE_MAP)  
hmmwv.SetTireType(chrono.ChTireModelType_PACEJKA)  
hmmwv.Initialize()


terrain = chrono.ChTerrain()
terrain.SetSize(100, 100)  
terrain.SetFriction(0.8)  
terrain.SetRestitution(0.3)  
material = chrono.ChMaterialSurface()
material.SetFriction(0.8)
material.SetRestitution(0.3)
terrain.SetMaterial(material)
terrain.Initialize()
system.Add(terrain)


system.Add(hmmwv.GetChassis())


driver = chrono.ChDriver()
driver.Initialize(hmmwv.GetDriverInputs())
hmmwv.SetDriver(driver)


rospy.init_node('chrono_simulation')
ros_manager = chrono_ros.ChRosManager(system)
ros_manager.Initialize()


pub_state = rospy.Publisher('vehicle_state', JointState, queue_size=10)
sub_clock = rospy.Subscriber('clock', rospy.Time, lambda msg: ros_manager.SynchronizeTime(msg))
sub_driver = rospy.Subscriber('driver_inputs', Float32MultiArray, lambda msg: driver_inputs_callback(msg, driver))

def driver_inputs_callback(msg, driver):
    
    driver_inputs = chrono.ChDriverInputs()
    driver_inputs.m_throttle = msg.data[0]
    driver_inputs.m_brake = msg.data[1]
    driver_inputs.m_steering = msg.data[2]
    driver.SetDriverInputs(driver_inputs)


step_size = 0.001  
while not rospy.is_shutdown():
    
    system.DoStepDynamics(step_size)
    
    
    driver.Synchronize()
    
    
    ros_manager.Synchronize()
    
    
    state_msg = JointState()
    state_msg.header.stamp = rospy.Time.now()
    state_msg.position = [hmmwv.GetChassis().GetPos().x, hmmwv.GetChassis().GetPos().y, hmmwv.GetChassis().GetPos().z,
                         hmmwv.GetChassis().GetRot().e0, hmmwv.GetChassis().GetRot().e1, hmmwv.GetChassis().GetRot().e2, hmmwv.GetChassis().GetRot().e3]
    pub_state.publish(state_msg)
    
    
    rospy.sleep(step_size)


ros_manager.Shutdown()
rospy.signal_shutdown('Simulation finished')