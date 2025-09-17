import pychrono as chrono
import pychrono.vehicle as veh
import rospy
from std_msgs.msg import Float32MultiArray


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
time_step = 0.001
system.SetTimestepperType(chrono.ChTimestepper.Type_UNSET)


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod.NSC)  
hmmwv.SetEngineType(veh.EngineModelType.REALISTIC)   
hmmwv.SetTireType(veh.TireModelType.PACEJKA)        
hmmwv.Initialize(system)


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.9)
material.SetRestitution(0.1)
plane = chrono.ChPlane()
plane.SetMaterialSurface(material)
system.Add(plane)


driver = veh.ChDriver()
hmmwv.SetDriver(driver)


rospy.init_node('hmmwv_sim')
pub = rospy.Publisher('/vehicle/state', Float32MultiArray, queue_size=10)
sub = rospy.Subscriber('/driver/input', Float32MultiArray, lambda msg: driver.SetThrottle(msg.data[0]).SetSteering(msg.data[1]))


current_time = 0.0
max_time = 10.0

while not rospy.is_shutdown() and current_time < max_time:
    driver.Synchronize(current_time)
    system.DoStepDynamics(time_step)
    current_time += time_step

    
    state = Float32MultiArray()
    state.data = [
        hmmwv.GetChassis().GetPos().x,
        hmmwv.GetChassis().GetPos().y,
        hmmwv.GetChassis().GetPos().z,
        hmmwv.GetChassis().GetPos_dt().Length()
    ]
    pub.publish(state)

    rospy.sleep(0.001)