Here is the code that meets the specifications:
```
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle
import numpy as np
import rospy
from std_msgs.msg import Int32


chrono.SetChronoDataPath("/path/to/chrono/data")


mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.5)


floor = chrono.ChBodyEasyBox(mysystem, 10, 1, 10, 1000, True, material)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)


box = chrono.ChBodyEasyBox(mysystem, 1, 1, 1, 1000, True, material)
box.SetPos(chrono.ChVectorD(0, 2, 0))
box.SetBodyFixed(False)


class MyROSHdlr(chrono.ChROSHandler):
    def __init__(self):
        super().__init__()
        self.pub = rospy.Publisher('my_topic', Int32, queue_size=10)

    def Advance(self, mstep):
        msg = Int32()
        msg.data = mstep
        self.pub.publish(msg)


ros_manager = chrono.ChROSManager()


clock_hdlr = chrono.ChROSClockHandler()
ros_manager.AddHandler(clock_hdlr)


body_hdlr = chrono.ChROSBODYHandler()
ros_manager.AddHandler(body_hdlr)


transform_hdlr = chrono.ChROSTransformHandler()
ros_manager.AddHandler(transform_hdlr)


ros_manager.RegisterHandler(MyROSHdlr())


app = chronoirr.ChVisualSystemIrrlicht()
app.SetWindowSize(800, 600)
app.SetWindowTitle("PyChrono Simulation")
app.SetTypicalLenght(1.0)
app.SetCamera(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(0, 1, 0))


driver = app.GetDriver()


while app.Run():
    
    mysystem.DoStepDynamics(0.01)

    
    ros_manager.Advance(mysystem.GetChTime())

    
    driver.BeginScene(True, True, True)
    driver.Render()
    driver.EndScene()
```
Note that you'll need to replace `"/path/to/chrono/data"` with the actual path to the Chrono data directory on your system. Also, make sure to install the necessary dependencies, including PyChrono, ROS, and Irrlicht.