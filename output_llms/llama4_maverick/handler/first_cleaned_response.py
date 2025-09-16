import pychrono as chrono
import pychrono.irrlicht as chronoirr
import rospy
from std_msgs.msg import Int32


chrono_sys = chrono.ChSystemNSC()


chrono_sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor.SetCollide(True)
floor_shape = chrono.ChBoxShape()
floor_shape.GetBox().SetLengths(chrono.ChVectorD(10, 1, 10))
floor.AddShape(floor_shape)
chrono_sys.Add(floor)


box = chrono.ChBody()
box.SetMass(1)
box.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
box.SetPos(chrono.ChVectorD(0, 2, 0))
box.SetCollide(True)
box_shape = chrono.ChBoxShape()
box_shape.GetBox().SetLengths(chrono.ChVectorD(1, 1, 1))
box.AddShape(box_shape)
chrono_sys.Add(box)


rospy.init_node('chrono_sim_ros', anonymous=True)
pub = rospy.Publisher('/chrono_sim_topic', Int32, queue_size=10)
rate = rospy.Rate(100)  


class CustomROSHandler:
    def __init__(self, publisher):
        self.publisher = publisher
        self.count = 0

    def Update(self):
        msg = Int32()
        msg.data = self.count
        self.publisher.publish(msg)
        self.count += 1

custom_handler = CustomROSHandler(pub)


while not rospy.is_shutdown():
    chrono_sys.DoStepDynamics(0.01)  
    custom_handler.Update()
    rate.sleep()




import pychrono as chrono
import rospy
from std_msgs.msg import Int32


chrono_sys = chrono.ChSystemNSC()


chrono_sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor.SetCollide(True)
floor_shape = chrono.ChBoxShape()
floor_shape.GetBox().SetLengths(chrono.ChVectorD(10, 1, 10))
floor.AddShape(floor_shape)
chrono_sys.Add(floor)


box = chrono.ChBody()
box.SetMass(1)
box.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
box.SetPos(chrono.ChVectorD(0, 2, 0))
box.SetCollide(True)
box_shape = chrono.ChBoxShape()
box_shape.GetBox().SetLengths(chrono.ChVectorD(1, 1, 1))
box.AddShape(box_shape)
chrono_sys.Add(box)


rospy.init_node('chrono_sim_ros', anonymous=True)
pub = rospy.Publisher('/chrono_sim_topic', Int32, queue_size=10)
rate = rospy.Rate(100)  


class CustomROSHandler:
    def __init__(self, publisher):
        self.publisher = publisher
        self.count = 0

    def Update(self):
        msg = Int32()
        msg.data = self.count
        self.publisher.publish(msg)
        self.count += 1

custom_handler = CustomROSHandler(pub)


while not rospy.is_shutdown():
    chrono_sys.DoStepDynamics(0.01)  
    custom_handler.Update()
    rate.sleep()