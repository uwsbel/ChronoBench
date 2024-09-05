import pychrono as chrono
import pychrono.ros as chroro
import rospy


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.2)


floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, -0.1, 0))
my_system.Add(floor)


box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 10, True, True, material)
box.SetPos(chrono.ChVectorD(0, 1, 0))
my_system.Add(box)


rospy.init_node('chrono_ros_node')


ros_manager = chroro.ChROSManager(my_system)


ros_manager.SetClockHandler(chroro.ChROSClockHandler())
ros_manager.SetBodyHandler(chroro.ChROSBodyHandler())
ros_manager.SetTransformHandler(chroro.ChROSTransformHandler())


class CustomROSHandler(chroro.ChROSHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, Int32, queue_size=10)

    def OnUpdate(self, data):
        message = Int32()
        message.data = int(data)  
        self.publisher.publish(message)


custom_handler = CustomROSHandler('/chrono_data')
ros_manager.RegisterHandler(custom_handler)


while not rospy.is_shutdown():
    
    my_system.DoStepDynamics(0.01)

    
    ros_manager.Update()

    
    custom_handler.OnUpdate(box.GetPos().y)

    
    rospy.rostime.wallsleep(0.01)