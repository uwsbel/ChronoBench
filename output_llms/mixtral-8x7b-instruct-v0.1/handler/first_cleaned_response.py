import pychrono as chrono
import pychrono.ros as chronoros
import pychrono.irrlicht as chronoirr
import pyrosbag as rosbag
import pyrosplan as rospr
import time
import math


chrono.SetChronoDataPath('path/to/chrono/data')
my_engine = chrono.ChSystemSolverAE()


my_engine.Set_G_acc(chrono.ChVector(0, -9.81, 0))
my_material = chrono.ChMaterialSurfaceNSC()
my_material.SetFriction(0.5)
my_material.SetRestitution(0.1)


floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, my_material, chrono.ChFrame(chrono.ChVector(0, -5, 0)))
my_engine.AddBody(floor)


box_shape = chrono.ChBox(1, 1, 1, chrono.ChVector(0, 1, 0))
box_body = chrono.ChBodyEasyBox(1, 1, 1, 500, my_material, chrono.ChFrame(chrono.ChVector(0, 5, 0)))
my_engine.AddBody(box_body)


class IntPublisher:
    def __init__(self, topic, rate):
        self.pub = rosbag.Publisher(topic, 'std_msgs/Int32', rate)
        self.counter = 0

    def next(self):
        self.counter += 1
        msg = std_msgs.msg.Int32()
        msg.data = self.counter
        self.pub.publish(msg)


ros_manager = chronoros.ChRosManager()
ros_manager.Initialize(my_engine)
ros_manager.AddClockHandler(chronoros.ChRosClock())
ros_manager.AddBodyHandler(chronoros.ChRosBody())
ros_manager.AddTransformHandler(chronoros.ChRosTransform())


ros_manager.AddCustomHandler(IntPublisher('topic_name', 100))


start_time = time.time()
while True:
    dt = min(0.01, time.time() - start_time)
    my_engine.DoStepDynamics(dt)
    ros_manager.Synchronize()
    start_time = time.time()