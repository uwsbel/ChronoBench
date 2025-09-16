import chrono
import chrono.ir_obs as ir_obs
import chrono.ir_obs_manager as ir_obs_manager
import chrono.ir_obs_handlers as ir_obs_handlers
import chrono.synchronization as sync
import rospy
from std_msgs.msg import Int32
import rospkg


chrono.init()


g = 9.81
chrono.ChSystemNSC.Set_G_acc(chrono.ChVectorD(0, 0, -g))


system = chrono.ChSystemNSC()


floor = chrono.ChBodyEasySFEM(chrono.ChBodyTypes.CONSTROYER, 1, chrono.ChVectorD(0, 0, 0))
system.AddBody(floor)


box = chrono.ChBodyEasySFEM(chrono.ChBodyTypes.BOX, 1, 1, 1)
system.AddBody(box)


box.SetPos(chrono.ChVectorD(0, 1, 0))


box.SetMass(1)
box.SetInertia(chrono.ChVectorD(1, 1, 1))


system.AddBody(box)


class IntegerMessageHandler(ir_obs_handlers.IRROS_Handler):
    def __init__(self, topic_name, topic_type):
        super().__init__(topic_name, topic_type)

    def Update(self, data):
        rospy.loginfo(f"Received message: {data.data}")
        self.publisher.publish(data)


rospy.init_node('pychrono_ros_handler', anonymous=True)


ros_manager = chrono.IRROS_Manager()
ros_manager.AddClock(chrono.IRROS_Clock())
ros_manager.AddBody(system)
ros_manager.AddTransform(chrono.IRROS_Transform())
ros_manager.AddBody(box)
ros_manager.AddHandler(IntegerMessageHandler('my_topic', rospy.Int32))


ros_manager.RegisterHandler(IntegerMessageHandler('my_topic', rospy.Int32))


while not rospy.is_shutdown():
    
    system.DoStepDynamics(1.0/60.0)
    ros_manager.Update()

    
    if rospy.Time.now().to_sec() % 1 < 0.01:
        ros_manager.Publish(chrono.ChTime(), rospy.Int32(42))

    
    rospy.sleep(0.01)


chrono.reset()