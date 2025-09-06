import pychrono as chrono
import pychrono.ros as chros
import rospy
from std_msgs.msg import Int32


class CustomIntPublisher(chros.ChROSHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.topic_name = topic_name
        self.publisher = None
        self.counter = 0

    def Initialize(self):
        if not rospy.get_node_uri():
            rospy.init_node('pychrono_node', anonymous=True)
        self.publisher = rospy.Publisher(self.topic_name, Int32, queue_size=10)
        return True

    def Publish(self, time):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1
        return True

def main():
    
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    
    material = chrono.ChContactMaterialNSC()
    material.SetFriction(0.4)
    material.SetRestitution(0.1)

    
    floor = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, material)
    floor.SetPos(chrono.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    system.Add(floor)

    
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
    box.SetPos(chrono.ChVector3d(0, 0, 2))
    system.Add(box)

    
    ros_manager = chros.ChROSPyManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(ros_manager, box, "box"))
    ros_manager.RegisterHandler(chros.ChROSTFHandler(ros_manager, 100))
    
    
    int_publisher = CustomIntPublisher("pychrono_counter")
    ros_manager.RegisterHandler(int_publisher)

    
    if not ros_manager.Initialize():
        print("ROS initialization failed!")
        return

    
    time_step = 0.01
    realtime_step = True
    frame_interval = 0.01  

    while ros_manager.NodeOK():
        
        system.DoStepDynamics(time_step)
        
        
        ros_manager.Update(system.GetChTime(), realtime_step)
        
        
        if realtime_step:
            ros_manager.SpinSome(frame_interval)

if __name__ == "__main__":
    main()