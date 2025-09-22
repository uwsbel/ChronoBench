import pychrono as chrono
import pychrono.ros as roschrono
import pychrono.irrlicht as chronoirr
import pyros_numpy as pyros
import rospy
import numpy as np


chrono.SetChronoDataPath('path/to/chrono/data')


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.1)


floor = system.AddBody(chrono.ChBodyEasyBox(10, 0.1, 10, 1000, chrono.ChFrameD(chrono.ChVectorD(0, -0.5, 0))))
floor.SetMaterialSurface(material)
floor.SetCollide(True)


box = system.AddBody(chrono.ChBodyEasyBox(1, 1, 1, 100, chrono.ChFrameD(chrono.ChVectorD(0, 5, 0))))
box.SetMaterialSurface(material)
box.SetCollide(True)
box.SetPos(chrono.ChVectorD(0, 2, 0))


def ros_handler(body, topic, msg_type):
    def publish_msg():
        quat = body.GetPos_dt().GetWxyz()
        pos = body.GetPos().ToArray()
        msg = pyros.geometry_msgs.PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        pub.publish(msg)

    pub = rospy.Publisher(topic, msg_type, queue_size=10)
    rospy.Timer(chrono.ChTime(0.01), publish_msg)


ros_manager = roschrono.ChSystemRosManager()
clock_handler = roschrono.ChRosClockHandler()
body_handler = roschrono.ChRosBodyHandler()
transform_handler = roschrono.ChRosTransformHandler()
ros_manager.AddHandler(clock_handler)
ros_manager.AddHandler(body_handler)
ros_manager.AddHandler(transform_handler)
ros_manager.AddHandler(ros_handler(box, '/box_pose', pyros.geometry_msgs.PoseStamped))


vis = chronoirr.ChIrrApp(system.GetSystem(), 'PyChrono ROS Simulation')
vis.AddLogo()
vis.AddTypicalSky()
vis.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChVectorD(0, 100, 0), chrono.ChVectorD(0, -1, 0), 400)
vis.SetTimestep(chrono.ChTime(1 / 240.0))
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('PyChrono ROS Simulation')
vis.Initialize()
vis.AddSystem(ros_manager)
vis.Start()