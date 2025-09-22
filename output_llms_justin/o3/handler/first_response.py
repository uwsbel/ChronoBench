#!/usr/bin/env python3
# ---------------------------------------------------------------------
# Chrono – ROS demo
#
# * Fixed concrete floor (body-fixed)
# * Dynamic wooden box (freely falling / sliding)
# * Full Chrono–ROS bridge:
#     – /clock          : simulation clock
#     – /box            : body state publisher (pose, twist, wrench)
#     – /tf             : world ↔ body transforms
# * Custom (user-defined) ROS handler that publishes an Int32 counter
# ---------------------------------------------------------------------

import time
import math
import rospy                                   # ROS Python (rospy)
from std_msgs.msg import Int32                 # message for custom publisher

import pychrono.core as chrono                 # Chrono core
# The ROS bridge lives in the “ros” Python sub-module that is created
# when Chrono is built with   -DCHRONO_ROS=ON
try:
    import pychrono.ros as chros
except ImportError:
    chros = None
    rospy.logwarn("Chrono was not compiled with the ROS bridge. "
                  "The simulation will still run, but no built-in "
                  "handlers (/clock, /tf, /body) will be produced.")


# ---------------------------------------------------------------------
# 1.  Custom handler that publishes consecutive integers
# ---------------------------------------------------------------------
class IntPublisher:
    """
    Very small helper that emulates the behaviour of a Chrono ROS
    handler: it owns a rospy.Publisher and exposes an update() method
    that we call once per simulation step.
    """
    def __init__(self, topic_name="/int_counter", queue_size=10):
        self._pub     = rospy.Publisher(topic_name, Int32, queue_size=queue_size)
        self._counter = 0

    def update(self):
        """
        Publish the current counter value and then increment it.
        """
        msg       = Int32()
        msg.data  = self._counter
        self._pub.publish(msg)
        self._counter += 1


# ---------------------------------------------------------------------
# 2.  Build the Chrono mechanical system (floor + box)
# ---------------------------------------------------------------------
def create_system():

    # ---- core system -------------------------------------------------
    sys = chrono.ChSystemSMC()                         # SMC = soft-contact
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))      # gravity

    # ---- material shared by all objects -----------------------------
    mat = chrono.ChMaterialSurfaceSMC()
    mat.SetYoungModulus(2.0e5)     # [Pa]
    mat.SetFriction(0.5)
    mat.SetRestitution(0.1)

    # ---- floor (large box, body-fixed) ------------------------------
    floor_size      = chrono.ChVectorD(4, 0.20, 4)     # [m]
    floor_density   = 1000                             # [kg/m³]

    floor = chrono.ChBodyEasyBox(floor_size.x,
                                 floor_size.y,
                                 floor_size.z,
                                 floor_density,        # density
                                 True,                 # visualize
                                 True,                 # collide
                                 mat)                  # contact mat
    floor.SetPos(chrono.ChVectorD(0, -floor_size.y / 2.0, 0))
    floor.SetBodyFixed(True)
    sys.Add(floor)

    # ---- movable box -------------------------------------------------
    box_size      = chrono.ChVectorD(0.5, 0.5, 0.5)
    box_density   = 700                                # wood approx.

    box = chrono.ChBodyEasyBox(box_size.x,
                               box_size.y,
                               box_size.z,
                               box_density,
                               True,
                               True,
                               mat)
    box.SetPos(chrono.ChVectorD(0, 1.0, 0))            # 1 m above floor
    sys.Add(box)

    return sys, floor, box


# ---------------------------------------------------------------------
# 3.  Attach built-in Chrono ROS handlers (clock, body, tf)
# ---------------------------------------------------------------------
def create_ros_bridge(system, floor, box):
    """
    Returns a ChROSBridge object if the ROS module is available.
    Otherwise returns None.
    """
    if chros is None:
        return None

    bridge = chros.ChROSBridge(system)                 # root bridge

    # /clock (std_msgs/Clock)
    bridge.AddROSPublisher(chros.ChROSClockHandler())

    # Body state publisher for the moving box
    # (publishes geometry_msgs/Pose|Twist|Wrench on /box)
    bridge.AddROSPublisher(chros.ChROSBodyHandler(box, "box"))

    # Static transform floor ↔ world
    bridge.AddROSPublisher(chros.ChROSTFHandler(floor, "floor"))

    # Dynamic transform box ↔ world
    bridge.AddROSPublisher(chros.ChROSTFHandler(box,  "box"))

    return bridge


# ---------------------------------------------------------------------
# 4.  Main loop: advance Chrono, update ROS, stay real-time
# ---------------------------------------------------------------------
def run_simulation():
    # -----------------------------------
    # ROS node initialisation
    # -----------------------------------
    rospy.init_node("chrono_ros_sim", anonymous=True)

    # -----------------------------------
    # Chrono system creation
    # -----------------------------------
    system, floor, box = create_system()

    # -----------------------------------
    # ROS bridge (+ custom int publisher)
    # -----------------------------------
    bridge      = create_ros_bridge(system, floor, box)
    int_handler = IntPublisher("/my_int_topic")

    # -----------------------------------
    # Real-time loop
    # -----------------------------------
    step_size  = 1.0 / 1000.0                     # 1 kHz ≈ 0.001 s
    realtime   = chrono.ChRealtimeStepTimer()     # helper that sleeps

    sim_time   = 0.0
    end_time   = 10.0                             # [s] demo length

    rospy.loginfo("Starting Chrono – ROS simulation …")
    while (not rospy.is_shutdown()) and (sim_time < end_time):

        # ---- 4.1 Dynamics step --------------------------------------
        system.DoStepDynamics(step_size)
        sim_time += step_size

        # ---- 4.2 Built-in bridge update ----------------------------
        if bridge is not None:
            bridge.Update()                       # pushes all handlers

        # ---- 4.3 Custom handler update -----------------------------
        int_handler.update()

        # ---- 4.4 Keep wall-clock pace ------------------------------
        realtime.Spin(step_size)

    rospy.loginfo("Simulation finished.")


# ---------------------------------------------------------------------
# 5.  Entry point
# ---------------------------------------------------------------------
if __name__ == "__main__":
    try:
        run_simulation()
    except rospy.ROSInterruptException:
        pass