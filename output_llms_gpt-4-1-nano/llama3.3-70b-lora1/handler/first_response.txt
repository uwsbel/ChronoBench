import pychrono as ch
import pychrono.ros as chros

import rclpy  # noqa
import rclpy.node  # noqa
import rclpy.parameter  # noqa
import rclpy.publisher  # noqa
import rclpy.subscription  # noqa
import rclpy.timer  # noqa
import rclpy.qos  # noqa

import std_msgs.msg as std_msg  # noqa

# Create Chrono system
system = ch.ChSystemNSC()
system.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

# Add a floor and box to the scene
phys_mat = ch.ChContactMaterialNSC()
phys_mat.SetFriction(0.5)

floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
floor.SetPos(ch.ChVector3d(0, 0, -1))
floor.SetFixed(True)
floor.SetName("floor")
system.Add(floor)

box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
box.SetPos(ch.ChVector3d(0, 0, 5))
box.SetName("box")
system.Add(box)

# Create ROS manager
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBroadcasterHandler(25))
ros_manager.RegisterHandler(chros.ChROSBodyHandler(  # noqa
    box, "/box", "~/box_pose",  # noqa
    "~/box_twist"  # noqa
))
ros_manager.Initialize()

# Create a custom ROS handler
class MyHandler(chros.ChROSHandler):
    def __init__(self):
        chros.ChROSHandler.__init__(self)

        self.msg = std_msg.Int()
        self.publisher = rclpy.publisher.Publisher(self, std_msg.Int, "~/my_topic")

    def Initialize(self):
        return

    def TickCallback(self, time):
        self.msg.data = int(time)
        self.publisher.publish(self.msg)
        return

ros_manager.RegisterHandler(MyHandler())

# Simulation loop
time = 0
time_step = 1e-3
time_end = 1.0

realtime_timer = ch.ChRealtimeStepTimer()
while time < time_end:
    system.DoStepDynamics(time_step)
    ros_manager.Update(time, time_step)
    time = time + time_step
    realtime_timer.Spin(time_step)