import pychrono as chrono
import pychrono.irrlicht as chronoirr
import time
import rospy
from std_msgs.msg import Int32

class CustomROSHelper(object):
    
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, Int32, queue_size=10)

    def publish_message(self, data):
        
        message = Int32()
        message.data = data
        self.publisher.publish(message)

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
    body_floor.SetMaterialSurface(material)
    body_floor.SetShapeBoxes(0.5, 0.1, 1)
    system.Add(body_floor)

    
    body_box = chrono.ChBodyEasy()
    body_box.SetPos(chrono.ChVectorD(0, 1, 0))
    body_box.SetMaterialSurface(material)
    body_box.SetShapeBoxes(0.1, 0.1, 0.1)
    system.Add(body_box)

    
    ros_helper = CustomROSHelper('chrono_data')

    
    ros_manager = chrono.ChRosManager()
    ros_manager.SetSystem(system)
    ros_manager.SetClockHandler(True)
    ros_manager.SetBodyHandler(True)
    ros_manager.SetTransformHandler(True)
    ros_manager.RegisterHelper(ros_helper)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('PyChrono with ROS')
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(0, 2, -3))
    vis.AddTypicalLights()

    
    time_step = 0.005
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        system.DoStepDynamics(time_step)

        
        ros_manager.Update()
        ros_helper.publish_message(int(body_box.GetPos().y() * 100)) 

        
        time.sleep(time_step)

    print("Simulation finished.")

if __name__ == "__main__":
    main()