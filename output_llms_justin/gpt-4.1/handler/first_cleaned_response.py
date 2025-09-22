import pychrono as chrono
import pychrono_ros as chros
import rospy
from std_msgs.msg import Int32
import time


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.6)
material.SetRestitution(0.1)


floor = chrono.ChBodyEasyBox(2, 2, 0.1, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, 0, -0.05))
floor.SetBodyFixed(True)
system.Add(floor)


box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 100, True, True, material)
box.SetPos(chrono.ChVectorD(0, 0, 0.5))
system.Add(box)


class MyIntPublisher(chros.ChROSHandler):
    def __init__(self, topic_name):
        super().__init__()
        self.pub = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.counter = 0

    def Tick(self, time):
        
        msg = Int32()
        msg.data = self.counter
        self.pub.publish(msg)
        self.counter += 1


rospy.init_node('chrono_ros_sim', anonymous=True)
ros_manager = chros.ChROSManager(system)


clock_handler = chros.ChROSClockHandler()
ros_manager.AddHandler(clock_handler)


body_handler = chros.ChROSBodyHandler()
body_handler.AddBody(box, "box")
ros_manager.AddHandler(body_handler)


tf_handler = chros.ChROSTFHandler()
tf_handler.AddBody(box, "box")
ros_manager.AddHandler(tf_handler)


int_pub_handler = MyIntPublisher('/my_integer_topic')
ros_manager.AddHandler(int_pub_handler)


timestep = 0.01
end_time = 2.0
sim_time = 0.0


rate = rospy.Rate(1.0 / timestep)
while not rospy.is_shutdown() and sim_time < end_time:
    system.DoStepDynamics(timestep)
    ros_manager.Update(sim_time)
    sim_time += timestep
    rate.sleep()

print("Simulation finished.")