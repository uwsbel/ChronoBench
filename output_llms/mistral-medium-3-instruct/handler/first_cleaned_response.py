import pychrono as chrono
import pychrono.ros as chrono_ros
import rospy
from std_msgs.msg import Int32


rospy.init_node('chrono_ros_simulation', anonymous=True)


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.4)
material.SetRestitution(0.2)


floor_body = chrono.ChBody()
floor_body.SetPos(chrono.ChVectorD(0, -1, 0))
floor_body.SetBodyFixed(True)
floor_body.GetVisualModel().AddBox(10, 1, 10, True, chrono.ChColor(0.5, 0.5, 0.5))
floor_body.GetCollisionModel().AddBox(10, 1, 10)
floor_body.GetCollisionModel().SetMaterialSurface(material)
system.Add(floor_body)


box_body = chrono.ChBody()
box_body.SetPos(chrono.ChVectorD(0, 2, 0))
box_body.SetMass(1.0)
box_body.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
box_body.GetVisualModel().AddBox(1, 1, 1, True, chrono.ChColor(0.8, 0.2, 0.2))
box_body.GetCollisionModel().AddBox(1, 1, 1)
box_body.GetCollisionModel().SetMaterialSurface(material)
system.Add(box_body)


class CustomIntPublisher(chrono_ros.ChRosHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.pub = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.counter = 0

    def Update(self):
        msg = Int32()
        msg.data = self.counter
        self.pub.publish(msg)
        self.counter += 1


ros_manager = chrono_ros.ChRosManager()


ros_manager.AddHandler(chrono_ros.ChRosClockHandler())
ros_manager.AddHandler(chrono_ros.ChRosBodyHandler())
ros_manager.AddHandler(chrono_ros.ChRosTransformHandler())


custom_handler = CustomIntPublisher('/chrono_counter')
ros_manager.AddHandler(custom_handler)


if True:
    from pychrono import irrlicht as chrono_irr
    vis = chrono_irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle('PyChrono ROS Simulation')
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(3, 2, 0))
    vis.AddTypicalLogo()
    vis.AddTypicalSky()
    vis.AddLightWithShadow(chrono.ChVectorD(3, 5, 0), chrono.ChVectorD(0, 0, 0), 3, 1, 5, 30, 512)


time_step = 0.01
simulation_time = 10.0
current_time = 0.0


while current_time < simulation_time:
    
    system.DoStepDynamics(time_step)

    
    ros_manager.Update()

    
    if 'vis' in locals():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    rospy.sleep(time_step)

    current_time += time_step


ros_manager.Shutdown()
if 'vis' in locals():
    vis.CloseDevice()