#!/usr/bin/env python3
"""
pychrono_ros_example.py

A PyChrono + ROS2 integration example:
  - Chrono NSC system with gravity
  - A fixed floor and a free box (ChBodyEasyBox)
  - Custom ROS handler publishes an Int32 message every step
  - Chrono–ROS manager publishes clock, box state, TFs, and our custom int
  - Real‐time simulation loop
"""

import time

# Chrono imports
import pychrono.core as chrono
import pychrono.ros as chrono_ros

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class CustomIntPublisher(chrono_ros.ChRosMsgHandler):
    """
    A custom Chrono‐ROS message handler that
    publishes an Int32 (e.g. currentStepCount or time→int) each step.
    """

    def __init__(self, node: Node, topic: str):
        super().__init__()
        self.node = node
        self.publisher = node.create_publisher(Int32, topic, 10)
        self.counter = 0

    def OnPublish(self, current_time: float):
        """
        Called by the Chrono–ROS manager each time it publishes.
        Here we simply publish a running counter or some function of time.
        """
        msg = Int32()
        # example: publish integer timestep count
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1


def main():
    # 1) Initialize ROS2
    rclpy.init()
    ros_node = rclpy.create_node('pychrono_ros2_sim')

    # 2) Create the Chrono NSC system with gravity
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # 3) Define a common material for contacts
    mat = chrono.ChMaterialSurfaceNSC()

    # 4) Add a fixed floor
    floor = chrono.ChBody()
    floor.SetName("ground")
    floor.SetBodyFixed(True)
    floor.SetMaterialSurface(mat)
    # Create a thin box shape for visualization
    floor_shape = chrono.ChBoxShape()
    floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(5, 5, 0.1)
    floor.AddVisualShape(floor_shape)
    floor.SetPos(chrono.ChVectorD(0, 0, -0.1))
    system.Add(floor)

    # 5) Add a free box
    box = chrono.ChBodyEasyBox(
        1.0,    # X size
        1.0,    # Y size
        1.0,    # Z size
        1000,   # density
        True,   # visualize
        True    # collide
    )
    box.SetName("box")
    box.SetMaterialSurface(mat)
    box.SetPos(chrono.ChVectorD(0, 0, 1.0))
    system.Add(box)

    # 6) Create the Chrono–ROS manager
    #    This will handle clock, body state, TFs, etc., for us
    manager = chrono_ros.ChRosManager.Create(ros_node)
    manager.AddSystem(system)             # link Chrono system
    manager.AddClockPublisher()           # publish /clock
    manager.AddBodyPublisher(box, "box")  # publish box pose & twist on topic "box"
    manager.AddTFPublisher()              # publish TFs for all bodies

    # 7) Register our custom Int32 publisher handler
    custom_int = CustomIntPublisher(ros_node, "my_custom_int")
    manager.RegisterHandler(custom_int)

    # 8) Simulation loop parameters
    dt = 1e-3   # integration step (s)
    rate = ros_node.create_rate(1.0 / dt)  # for real‐time pacing

    # 9) Main loop: step Chrono, let manager publish, spin ROS, sleep
    while rclpy.ok():
        system.DoStepDynamics(dt)

        # let Chrono–ROS manager trigger all its handlers
        manager.OnAdvance(system.GetChTime())

        # spin ROS once to handle publishers/subscribers
        rclpy.spin_once(ros_node, timeout_sec=0)

        # sleep to maintain real‐time
        rate.sleep()

    # 10) Clean up
    rclpy.shutdown()


if __name__ == "__main__":
    main()