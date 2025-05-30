import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import pychrono.sensor as sensor
import pychrono.vehicle as vehicle

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Import PyChrono ROS integration module
import pychrono_ros as chrono_ros

import time

class CustomROSHandler(chrono_ros.ChHandler):
    def __init__(self, node, topic_name):
        super().__init__()
        self.node = node
        self.publisher = node.create_publisher(Int32, topic_name, 10)
        self.message_counter = 0

    def OnPostStep(self, system, time):
        # Publish an integer message (for example, step count)
        msg = Int32()
        msg.data = self.message_counter
        self.publisher.publish(msg)
        self.message_counter += 1


class ROSNode(Node):
    def __init__(self):
        super().__init__('chrono_ros_node')


def main():
    # Initialize ROS 2
    rclpy.init()
    ros_node = ROSNode()

    # 1. Initialize PyChrono system and set gravity
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # 2. Physical material properties
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.6)
    material.SetRestitution(0.1)
    material.SetDampingF(0.01)

    # 3. Add fixed floor
    floor = chrono.ChBody()
    floor.SetBodyFixed(True)
    floor_shape = chrono.ChBoxShape()
    floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(5, 5, 0.1)
    floor.GetCollisionModel().ClearModel()
    floor.GetCollisionModel().AddBox(material, 5, 5, 0.1)
    floor.GetCollisionModel().BuildModel()
    floor.SetPos(chrono.ChVectorD(0, 0, -0.1))
    floor.SetCollide(True)
    floor.AddAsset(floor_shape)
    system.Add(floor)

    # 4. Add movable box
    box = chrono.ChBody()
    box.SetBodyFixed(False)
    box_shape = chrono.ChBoxShape()
    box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.5, 0.5, 0.5)
    box.GetCollisionModel().ClearModel()
    box.GetCollisionModel().AddBox(material, 0.5, 0.5, 0.5)
    box.GetCollisionModel().BuildModel()
    box.SetPos(chrono.ChVectorD(0, 0, 1))
    box.SetMass(1.0)
    box.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    box.SetCollide(True)
    box.AddAsset(box_shape)
    system.Add(box)

    # 5. Initialize the Chrono-ROS manager
    chrono_ros_mgr = chrono_ros.ChROSManager()
    chrono_ros_mgr.SetChronoSystem(system)

    # Setup core handlers
    chrono_ros_mgr.AddChronoTime()
    chrono_ros_mgr.AddChronoBodies()
    chrono_ros_mgr.AddChronoTransforms()

    # Register custom ROS handler publishing integer messages
    custom_handler = CustomROSHandler(ros_node, 'chrono_custom_int')
    chrono_ros_mgr.AddHandler(custom_handler)

    # Create a clock for time stepping
    time_step = 0.01
    realtime_timer = chrono.ChRealtimeStepTimer()

    # Simulation loop parameters
    simulation_duration = 5.0  # seconds
    sim_time = 0.0

    while rclpy.ok() and sim_time < simulation_duration:
        # Advance simulation by time_step
        system.DoStepDynamics(time_step)
        sim_time += time_step

        # Update ROS communication (call handler callbacks)
        chrono_ros_mgr.Update()

        # Keep ROS node spinning to handle the publish queue
        rclpy.spin_once(ros_node, timeout_sec=0)

        # Maintain realtime pace
        realtime_timer.Spin(time_step)

    # Shutdown ROS
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()