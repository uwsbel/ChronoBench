import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import rospy
from std_msgs.msg import Int32


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
material.SetDampingF(0.2)
material.SetCompliance(0.0001)
material.SetComplianceT(0.0001)


floor = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, material)
floor.SetPos(chrono.ChVectorD(0, -5, 0))
floor.SetBodyFixed(True)
sys.Add(floor)


box = chrono.ChBodyEasyBox(sys, 1, 1, 1, 10, True, material)
box.SetPos(chrono.ChVectorD(0, 5, 0))
sys.Add(box)


class CustomROSHändler:
    def __init__(self):
        self.pub = rospy.Publisher('custom_topic', Int32, queue_size=10)

    def Update(self):
        msg = Int32()
        msg.data = 1  
        self.pub.publish(msg)


rospy.init_node('pychrono_simulation')


ros_manager = chronoirr.ChVisualSystemIrrlicht()


ros_manager.AddTypicalClock()
ros_manager.AddTypicalBody(sys)
ros_manager.AddTypicalTransform(sys)


custom_handler = CustomROSHändler()
ros_manager.RegisterTypicalHandler(custom_handler)


while not rospy.is_shutdown():
    
    sys.DoStepDynamics(0.01)

    
    ros_manager.BeginScene()
    ros_manager.DrawAll()
    ros_manager.EndScene()

    
    rospy.sleep(0.01)