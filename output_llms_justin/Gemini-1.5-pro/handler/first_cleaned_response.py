import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import pychrono.robot as chronorobot
import rospy 


from std_msgs.msg import Int32

class CustomRosHandler(chronorobot.RosChSystem):
    def __init__(self, step_size):
        chronorobot.RosChSystem.__init__(self, step_size)

        
        self.pub = rospy.Publisher('my_topic', Int32, queue_size=10)

    def update(self):
        
        msg = Int32()
        msg.data = 123  
        self.pub.publish(msg)


system = chrono.ChSystemNSC()
system.Set_G(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.2)


floor = chrono.ChBodyEasyBox(10, 1, 10, 1000)
floor.SetBodyFixed(True)
floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
floor.GetCollisionModel().ClearModel()
floor.GetCollisionModel().AddBox(material, 5, 0.5, 5)
floor.GetCollisionModel().BuildModel()
system.Add(floor)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVectorD(0, 1, 0))
box.GetCollisionModel().ClearModel()
box.GetCollisionModel().AddBox(material, 0.5, 0.5, 0.5)
box.GetCollisionModel().BuildModel()
system.Add(box)


rospy.init_node('chrono_sim')


ros_manager = chronorobot.RosChSystemManager(system)
ros_manager.AddClockHandler()
ros_manager.AddBodyHandler(box, "box")
ros_manager.AddTfHandler(chrono.ChFrameD(), "world")


custom_handler = CustomRosHandler(1e-3)  
ros_manager.AddHandler(custom_handler)


time_step = 1e-3
while not rospy.is_shutdown():
    
    system.DoStepDynamics(time_step)

    
    ros_manager.update()

    
    rospy.sleep(time_step)