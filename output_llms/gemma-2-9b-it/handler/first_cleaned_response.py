import pychrono as chrono
import pychrono_ros as ros_chrono
import rospy


rospy.init_node('pychrono_ros_example')


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.2)


floor = chrono.ChBodyEasyBox(10, 10, 0.1, material)
floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
floor.SetBodyFixed(True)
system.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, material)
box.SetPos(chrono.ChVectorD(0, 1, 0))
system.Add(box)


ros_manager = ros_chrono.RosManager(system)


class MyRosHandler(ros_chrono.RosBodyHandler):
    def __init__(self, body):
        super().__init__(body)
        self.topic = rospy.Publisher('my_topic', rospy.Int32, queue_size=10)

    def update(self):
        self.topic.publish(int(self.body.GetPos().y))


ros_manager.RegisterHandler(MyRosHandler(box))


ros_manager.AddClockHandler()
ros_manager.AddBodyHandler(box)
ros_manager.AddTransformHandler(box)


while not rospy.is_shutdown():
    system.DoStepDynamics(chrono.ChTime(0.01))  
    ros_manager.RosUpdate()