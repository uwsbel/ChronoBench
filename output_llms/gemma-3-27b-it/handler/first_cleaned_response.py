import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chronoros
import time
import rospy
from std_msgs.msg import Int32

class CustomRosHandler(chronoros.ChRosHandler):
    
    def __init__(self, topic_name):
        super().__init__()
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, Int32, queue_size=10)

    def DoPublish(self):
        
        msg = Int32()
        msg.data = chrono.ChTime().getTime()  
        self.publisher.publish(msg)
        rospy.loginfo(f"Published time: {msg.data} to topic: {self.topic_name}")


def main():
    

    
    rospy.init_node('pychrono_ros_sim', anonymous=True)

    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.4)
    material.SetRestitution(0.2)

    
    body_floor = chrono.ChBodyEasy()
    body_floor.SetBodyFixed(True)
    body_floor.SetPos(chrono.ChVectorD(0, -1, 0))
    body_floor.SetShapeBoxes(0.5, 0.1, 1)
    body_floor.SetMaterialSurface(material)
    system.Add(body_floor)

    
    body_box = chrono.ChBodyEasy()
    body_box.SetPos(chrono.ChVectorD(0, 1, 0))
    body_box.SetShapeBoxes(0.2, 0.2, 0.2)
    body_box.SetMaterialSurface(material)
    body_box.SetMass(1.0)
    system.Add(body_box)

    
    ros_topic_name = "/chrono_time"
    custom_handler = CustomRosHandler(ros_topic_name)

    
    ros_manager = chronoros.ChRosManager()

    
    ros_manager.AddChronoSystem(system)
    ros_manager.AddClockHandler()
    ros_manager.AddBodyHandler(body_box)
    ros_manager.AddTransformHandler(body_box)

    
    ros_manager.AddCustomHandler(custom_handler)

    
    ros_manager.Initialize()

    
    time_step = 0.005
    while True:
        
        system.DoStepDynamics(time_step)

        
        ros_manager.Update()

        
        sleep_time = time_step
        if time.time() - ros_manager.GetLastTime() < sleep_time:
            time.sleep(sleep_time - (time.time() - ros_manager.GetLastTime()))

        
        if rospy.is_shutdown():
            break

    
    rospy.signal_shutdown("Simulation finished.")

if __name__ == "__main__":
    main()