import pychrono as chrono
import pychrono.ros as chros
import rospy
from std_msgs.msg import Int32

class CustomROSPublisher(chros.ChROSHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.counter = 0

    def OnInitialize(self, interface, verbose):
        if verbose:
            rospy.loginfo("Custom ROS publisher initialized.")
        return True

    def OnAdvance(self, time):
        self.publisher.publish(Int32(self.counter))
        self.counter += 1
        return True

def main():
    
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.2)
    material.SetRestitution(0.1)

    
    floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, material)
    floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
    sys.Add(floor)

    
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
    box.SetPos(chrono.ChVectorD(0, 5, 0))
    sys.Add(box)

    
    rospy.init_node('chrono_simulation', anonymous=True)

    
    ros_manager = chros.ChROSManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  
    ros_manager.RegisterHandler(chros.ChROSTransformHandler(box, "box_link"))  

    
    custom_publisher = CustomROSPublisher("custom_topic")
    ros_manager.RegisterHandler(custom_publisher)

    
    ros_manager.Initialize()

    
    time_step = 0.01
    while not rospy.is_shutdown():
        sys.DoStepDynamics(time_step)
        current_time = sys.GetChTime()
        ros_manager.Update(current_time, time_step)

if __name__ == "__main__":
    main()