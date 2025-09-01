import chrono.chrono as c
import chrono.ros as ros
import rospy
from std_msgs.msg import Int32


c.Initialize()


sys = c.ChSystemSMC()
sys.Set_Gravity(c.ChVectorD(0, -9.81, 0))


material_floor = c.ChMaterialSurfaceNSC()
material_floor.SetFriction(0.6)
material_box = c.ChMaterialSurfaceNSC()
material_box.SetFriction(0.6)


floor = c.ChBodyEasyBox(1, 1, 0.1, material_floor)
floor.SetPos(c.ChVectorD(0, -0.5, 0))
floor.SetBodyFixed(True)
sys.Add(floor)


box = c.ChBodyEasyBox(0.2, 0.2, 0.2, material_box)
box.SetPos(c.ChVectorD(0, 0.5, 0))
box.SetMass(1)
box.SetInertiaXX(c.ChVectorD(0.008, 0.008, 0.016))
sys.Add(box)


class CustomRosHandler(ros.ChRosNodeHandler):
    def __init__(self, name):
        ros.ChRosNodeHandler.__init__(self, name)
        self.pub = self.create_publisher('custom_topic', Int32, queue_size=10)

    def Update(self, my_system, my_items):
        msg = Int32()
        msg.data = int(box.GetPos().y)
        self.pub.publish(msg)


rospy.init_node('pychrono_ros_node')
ros_manager = ros.ChRosManager()
ros_manager.Initialize('pychrono_ros_node')


ros_manager.RegisterClock()
ros_manager.RegisterBody(floor.GetName(), floor)
ros_manager.RegisterBody(box.GetName(), box)
ros_manager.RegisterTransform(box.GetName(), floor.GetName())


custom_handler = CustomRosHandler('custom_handler')
ros_manager.RegisterHandler(custom_handler)


while not rospy.is_shutdown():
    sys.DoStepDynamics(0.01)
    ros_manager.Update()