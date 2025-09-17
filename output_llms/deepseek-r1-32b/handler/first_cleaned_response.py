importchrono
import rospy
from std_msgs.msg import Int32


rospy.init_node('chrono_simulation_node')


system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, -9.81, 0))


floor = chrono.ChCollisionModel()
floor.SetBody(chrono.ChBody())
floor.GetBody().SetPos(chrono.ChVectorD(0, 0, 0))
floor.GetBody().SetBodyFixed(True)
floor.GetBody().SetCollide(True)
system.Add(floor)


box = chrono.ChBody()
box.SetMass(1.0)
box.SetPos(chrono.ChVectorD(0, 1, 0))
box.SetCollide(True)
box.GetCollisionModel().AddBox(chrono.ChVectorD(0.5, 0.5, 0.5))
system.Add(box)


class CustomRosHandler(chrono.ChRosHandler):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher = rospy.Publisher('chrono_topic', Int32, queue_size=10)
    
    def OnRosPublish(self):
        msg = Int32()
        msg.data = 42  
        self.publisher.publish(msg)


ros_manager = chrono.ChRosManager(system)
ros_manager.SetVerbose(False)
ros_manager.AddHandler(chrono.ChRosHandlerClock())
ros_manager.AddHandler(chrono.ChRosHandlerBody('box', box))
ros_manager.AddHandler(chrono.ChRosHandlerTransform('floor', floor.GetBody()))
ros_manager.AddHandler(CustomRosHandler('custom_handler'))


dt = 0.001
time = 0
rate = rospy.Rate(1000/dt)

while not rospy.is_shutdown():
    system.DoStepDynamics(dt)
    ros_manager.OnStep()
    time += dt
    if time > 10:  
        break
    rate.sleep()

rospy.spin()