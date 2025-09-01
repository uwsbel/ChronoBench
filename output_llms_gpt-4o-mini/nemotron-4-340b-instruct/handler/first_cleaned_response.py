import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chronoros
import rospy
from std_msgs.msg import Int32


chrono.ChSystemSMC()
system = chrono.ChSystem()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))



material = chrono.ChMaterialSurfaceSMC()
material.Set_YoungModulus(1e6)
material.Set_PoissonRatio(0.3)
material.Set_Friction(0.3)


floor_body = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True, material)
floor_body.SetPos(chrono.ChVectorD(0, -0.1, 0))
floor_body.SetBodyFixed(True)
system.Add(floor_body)


box_body = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1, True, True, material)
box_body.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(box_body)



box_body.SetPos_dt(chrono.ChVectorD(0, 0, 0))


class CustomROSHandler(chronoros.ChRosPublisher):
    def __init__(self, topic_name):
        super().__init__(topic_name, Int32)

    def Publish(self, time):
        msg = Int32()
        msg.data = int(time * 1000)  
        self.pub.publish(msg)


ros_manager = chronoros.ChRosManager(system)
ros_manager.AddClockHandler()
ros_manager.AddBodyHandler()
ros_manager.AddTransformHandler()


custom_handler = CustomROSHandler('/custom_topic')
ros_manager.RegisterPublisher(custom_handler)


rospy.init_node('pychrono_simulation')


timestep = 0.01
simulation_time = 0

while ros_manager.Ok():
    
    system.DoStepDynamics(timestep)

    
    ros_manager.Update(simulation_time)

    
    chrono.ChTime().Sleep(timestep)

    simulation_time += timestep