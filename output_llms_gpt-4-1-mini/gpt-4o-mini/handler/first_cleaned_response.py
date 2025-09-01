import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import rospy
from std_msgs.msg import Int32


class CustomROSHandler:
    def __init__(self):
        
        rospy.init_node('chrono_ros_node', anonymous=True)
        self.publisher = rospy.Publisher('chrono_topic', Int32, queue_size=10)
        self.rate = rospy.Rate(10)  

    def publish_message(self, message):
        self.publisher.publish(message)
        self.rate.sleep()


chrono.SetChronoDataPath('/path/to/chrono/data/')  
system = chrono.ChSystemNSC()


gravity = chrono.ChVectorD(0, -9.81, 0)
system.Set_G_acc(gravity)


floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -0.05, 0))
floor.SetBodyFixed(True)
system.Add(floor)


box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
box.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(box)


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
box.SetMaterialSurface(material)


ros_handler = CustomROSHandler()


while not rospy.is_shutdown():
    
    time_step = 0.01  
    system.DoStepDynamics(time_step)

    
    box_position = box.GetPos()
    ros_handler.publish_message(int(box_position.y))

    
    print(f"Simulation Time: {system.GetChTime():.2f} s")


rospy.signal_shutdown("Simulation ended.")