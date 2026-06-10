#!/usr/bin/env python3
"""
PyChrono + ROS simulation example.

This script:
  1. Initializes a Chrono physical system with gravity and a contact material.
  2. Adds a fixed floor and a movable box.
  3. Sets up the Chrono‑ROS manager with clock, body, and transform handlers.
  4. Registers a user‑defined handler that publishes an integer message each step.
  5. Runs the simulation in real‑time, advancing the dynamics and ROS communication
     together.
"""

# ----------------------------------------------------------------------
# 1.  Imports
# ----------------------------------------------------------------------
import rospy
import std_msgs.msg                     # for Int64
import pychrono.core as chrono          # core Chrono classes
import pychrono.ros as chronoros       # ROS bridge for Chrono

# ----------------------------------------------------------------------
# 2.  Custom ROS handler that publishes an integer message
# ----------------------------------------------------------------------
class CustomIntHandler(chronoros.ChROSBaseHandler):
    """
    Example of a user‑defined ROS handler.
    It publishes the current simulation time (in milliseconds) as an Int64.
    """
    def __init__(self, topic: str):
        super().__init__()
        # ROS publisher – queue size 10 is sufficient for a simple example
        self.pub = rospy.Publisher(topic, std_msgs.msg.Int64, queue_size=10)

    def broadcast(self, time: float):
        """Called by the Chrono‑ROS manager at each simulation step."""
        msg = std_msgs.msg.Int64()
        msg.data = int(time * 1000)          # convert seconds → ms
        self.pub.publish(msg)


# ----------------------------------------------------------------------
# 3.  Helper to create a simple box body (used for floor and movable box)
# ----------------------------------------------------------------------
def make_box(name: str,
             width: float, height: float, depth: float,
             density: float,
             material: chrono.ChMaterialSurface,
             fixed: bool,
             pos: chrono.ChVectorD) -> chrono.ChBody:
    """
    Convenience factory for a box-shaped rigid body.
    """
    body = chrono.ChBodyEasyBox(width, height, depth,
                                density, material)
    body.SetName(name)
    body.SetPos(pos)
    body.SetFixed(fixed)
    return body


# ----------------------------------------------------------------------
# 4.  Main simulation routine
# ----------------------------------------------------------------------
def main():
    # ------------------------------------------------------------------
    # 4.1  ROS initialization
    # ------------------------------------------------------------------
    rospy.init_node("chrono_ros_example", anonymous=True)
    rospy.loginfo("Chrono‑ROS example node started")

    # ------------------------------------------------------------------
    # 4.2  Create the Chrono physical system
    # ------------------------------------------------------------------
    system = chrono.ChSystemNSC()               # non‑smooth contact (rigid)
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))   # gravity

    # Global simulation parameters
    step_size = 0.001            # 1 ms timestep
    end_time = 20.0              # stop after 20 s (set to 0 for infinite loop)

    # ------------------------------------------------------------------
    # 4.3  Define a contact material (shared by floor and box)
    # ------------------------------------------------------------------
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.3)
    material.SetRestitution(0.1)

    # ------------------------------------------------------------------
    # 4.4  Add a fixed floor
    # ------------------------------------------------------------------
    floor = make_box(
        name="floor",
        width=10.0, height=0.2, depth=10.0,
        density=1000.0,
        material=material,
        fixed=True,
        pos=chrono.ChVectorD(0.0, -0.1, 0.0)   # centre at y = -0.1 (just below y = 0)
    )
    system.Add(floor)

    # ------------------------------------------------------------------
    # 4.5  Add a movable box (starts 2 m above the floor)
    # ------------------------------------------------------------------
    box = make_box(
        name="box",
        width=0.5, height=0.5, depth=0.5,
        density=100.0,
        material=material,
        fixed=False,
        pos=chrono.ChVectorD(0.0, 2.0, 0.0)   # 2 m high
    )
    # Optional: give it a small initial velocity so it begins moving
    box.SetVel(chrono.ChVectorD(0.1, 0.0, 0.0))
    system.Add(box)

    # ------------------------------------------------------------------
    # 4.6  Set up the Chrono‑ROS manager
    # ------------------------------------------------------------------
    manager = chronoros.ChROSManager()

    # 4.6.1  Clock handler – publishes /clock (simulation time)
    clock_hdl = chronoros.ChROSClockHandler()
    manager.add_handler(clock_hdl)

    # 4.6.2  Body handler – publishes the box state on a custom topic
    body_hdl = chronoros.ChROSSBodyHandler(box, "/box/state")
    manager.add_handler(body_hdl)

    # 4.6.3  Transform handler – publishes TF from "world" → "box"
    tf_hdl = chronoros.ChROSTransformHandler()
    tf_hdl.set_parent_frame("world")
    tf_hdl.add_body("box", box)
    manager.add_handler(tf_hdl)

    # 4.6.4  Register the custom integer handler
    int_hdl = CustomIntHandler(topic="/chrono/sim_time_ms")
    manager.add_handler(int_hdl)

    # Initialise the manager (creates publishers,advertisements, etc.)
    manager.start()

    # ------------------------------------------------------------------
    # 4.7  Real‑time sleeper – keeps the loop wall‑time aligned with sim‑time
    # ------------------------------------------------------------------
    sleeper = chrono.ChRealtimeSleeper()

    # ------------------------------------------------------------------
    # 4.8  Simulation loop
    # ------------------------------------------------------------------
    sim_time = 0.0
    while not rospy.is_shutdown():
        # Stop if we have reached the desired end time
        if end_time > 0.0 and sim_time >= end_time:
            rospy.loginfo("Reached end time – stopping simulation.")
            break

        # Advance the dynamics by one step
        system.DoStepDynamics(step_size)
        sim_time += step_size

        # Send ROS messages (clock, body, TF, custom)
        manager.update(sim_time)

        # Sleep to keep real‑time (wall‑clock) aligned with simulation time
        sleeper.sleep(step_size)

    rospy.loginfo("Simulation loop terminated – shutting down.")


# ----------------------------------------------------------------------
# 5.  Entry point
# ----------------------------------------------------------------------
if __name__ == "__main__":
    main()