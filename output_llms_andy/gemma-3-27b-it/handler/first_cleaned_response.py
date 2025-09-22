import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import rospy
from std_msgs.msg import Int32

class ROSHandler(chrono.ChFunction):
    
    def __init__(self, topic_name, value):
        chrono.ChFunction.__init__(self)
        self.topic_name = topic_name
        self.value = value
        self.pub = rospy.Publisher(self.topic_name, Int32, queue_size=10)

    def Execute(self, chrono_time):
        msg = Int32()
        msg.data = self.value
        self.pub.publish(msg)
        rospy.loginfo("Published: %s", str(self.value))
        return True

def main():
    
    rospy.init_node('chrono_ros_example', anonymous=True)

    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.4)
    material.SetRestitution(0.2)

    
    body_floor = chrono.ChBodyEasy()
    body_floor.SetBodyFixed(True)
    body_floor.SetPos(chrono.ChVectorD(0, -1, 0))
    body_floor.SetMaterialSurface(material)
    body_floor.AddBox(10, 0.2, 10)
    system.Add(body_floor)

    
    body_box = chrono.ChBodyEasy()
    body_box.SetPos(chrono.ChVectorD(0, 2, 0))
    body_box.SetMaterialSurface(material)
    body_box.AddBox(1, 1, 1)
    system.Add(body_box)

    
    ros_handler = ROSHandler("/chrono_data", 42)

    
    ros_manager = chrono.ChRosManager()
    ros_manager.Set_Clock_Rate(30)  
    ros_manager.Set_Body_Publish_Rate(10) 
    ros_manager.Set_Transform_Publish_Rate(10) 
    ros_manager.Add_Handler(ros_handler)
    ros_manager.Initialize(system)

    
    time_step = 0.005
    while True:
        system.DoStepDynamics(time_step)
        ros_manager.Update()
        rospy.Rate(1/time_step).sleep() 

if __name__ == "__main__":
    main()