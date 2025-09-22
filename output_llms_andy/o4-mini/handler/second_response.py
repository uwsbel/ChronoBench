import pychrono as ch
import pychrono.ros as chros

import rclpy
from rclpy.publisher import Publisher
from std_msgs.msg import String


class MyCustomHandler(chros.ChROSHandler):
    """
    This custom handler will publish string messages to a specified ROS topic.
    """
    def __init__(self, topic: str):
        super().__init__(1)            # publish at 1 Hz
        self.topic = topic
        self.publisher: Publisher = None
        self.ticker = 0
        self.message = "Hello, world! At time: "

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Create the ROS publisher."""
        print(f"[MyCustomHandler] Creating publisher on topic '{self.topic}' …")
        self.publisher = interface.GetNode().create_publisher(
            String, self.topic, 1
        )
        return True

    def Tick(self, time: float):
        """Every Tick, publish 'Hello, world! At time: <ticker>'."""
        payload = self.message + str(self.ticker)
        print(f"[MyCustomHandler] Publishing: '{payload}'")
        msg = String()
        msg.data = payload
        self.publisher.publish(msg)
        self.ticker += 1


def main():
    # ------------------------------------------------------------------------------
    # 1) Initialize ROS 2
    # ------------------------------------------------------------------------------
    rclpy.init()

    # ------------------------------------------------------------------------------
    # 2) Create the Chrono system and set gravity
    # ------------------------------------------------------------------------------
    sys = ch.ChSystemNSC()
    sys.Set_G_acc(ch.ChVector3d(0, 0, -9.81))

    # Contact material
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    # ------------------------------------------------------------------------------
    # 3) Create a fixed floor
    # ------------------------------------------------------------------------------
    floor = ch.ChBodyEasyBox(
        10, 10, 1,     # size
        1000,          # density
        True, True, phys_mat
    )
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    # ------------------------------------------------------------------------------
    # 4) Create a free box
    # ------------------------------------------------------------------------------
    box = ch.ChBodyEasyBox(
        1, 1, 1,
        1000,
        True, True, phys_mat
    )
    box.SetPos(ch.ChVector3d(0, 0, 5))
    # Use Chrono’s Python helper to build a quaternion from (angle, axis)
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    sys.Add(box)

    # ------------------------------------------------------------------------------
    # 5) Set up ROS‐Chrono manager and handlers
    # ------------------------------------------------------------------------------
    ros_manager = chros.ChROSPythonManager()

    # Publish the ROS clock
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    # Publish the box’s pose & twist at 25 Hz
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, box, "~/box")
    )

    # Publish a transform between floor and box at 30 Hz
    tfh = chros.ChROSTFHandler(30)
    tfh.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tfh)

    # Add our custom string‐publisher
    my_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(my_handler)

    # Initialize all ROS handlers (this also spins up the ROS node)
    ros_manager.Initialize()

    # ------------------------------------------------------------------------------
    # 6) Main simulation loop
    # ------------------------------------------------------------------------------
    t_end = 30.0
    dt = 1e-3
    realtime_timer = ch.ChRealtimeStepTimer()

    while sys.GetChTime() < t_end:
        sys.DoStepDynamics(dt)
        current_time = sys.GetChTime()

        if not ros_manager.Update(current_time, dt):
            print("[main] ROS manager signaled a shutdown.")
            break

        realtime_timer.Spin(dt)

    # ------------------------------------------------------------------------------
    # 7) Clean up
    # ------------------------------------------------------------------------------
    ros_manager.Shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()