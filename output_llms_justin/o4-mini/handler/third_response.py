import pychrono as ch
import pychrono.irrlicht as irr
import pychrono.ros as chros

import rclpy
from std_msgs.msg import Int64

#
# Custom ROS handler that publishes an integer counter at a given rate.
#
class MyCustomHandler(chros.ChROSHandler):
    def __init__(self, topic: str, publish_rate: float):
        # initialize base with our desired rate
        super().__init__(publish_rate)
        self.topic = topic
        self.publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        # create a ROS2 publisher on the given topic
        print(f"[MyCustomHandler] Creating publisher on '{self.topic}' ...")
        node = interface.GetNode()
        self.publisher = node.create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time: float):
        # this is called at our publish_rate
        msg = Int64()
        msg.data = self.ticker
        print(f"[MyCustomHandler] Publishing {msg.data}")
        self.publisher.publish(msg)
        self.ticker += 1


def main():
    # 1) Initialize rclpy
    rclpy.init()

    # 2) Create Chrono system
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

    # 3) Define material for NSC contact
    mat = ch.ChMaterialSurfaceNSC()
    mat.SetFriction(0.5)

    # 4) Create floor
    floor = ch.ChBodyEasyBox(
        10, 10, 1,        # dimensions
        1000,             # density
        True, True, mat   # visualize, collide, material
    )
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    # set a texture (uses Chrono data folder, adjust path as needed)
    floor.SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)

    # 5) Create a box
    box = ch.ChBodyEasyBox(
        1, 1, 1,
        1000,
        True, True, mat
    )
    box.SetPos(ch.ChVector3d(0, 0, 5))
    # note the correct quaternion factory in PyChrono:
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    box.SetTexture(ch.GetChronoDataFile("textures/wood.jpg"))
    sys.Add(box)

    # 6) Create the Irrlicht visualization application
    app = irr.ChIrrApp(
        sys,
        "PyChrono + ROS2 + Irrlicht Demo",
        irr.dimension2du(1024, 768)
    )
    app.AddTypicalSky()
    app.AddTypicalCamera(
        irr.vector3df(0, -10, 5),  # camera position
        irr.vector3df(0, 0, 0)     # look-at
    )
    app.AddTypicalLights()

    # bind and update all assets so that textures, etc. are ready
    app.AssetBindAll()
    app.AssetUpdateAll()

    # 7) Create and initialize the ROS manager + handlers
    publish_rate = 10.0  # Hz for all ROS topics

    ros_manager = chros.ChROSPythonManager()

    # a) clock
    ros_manager.RegisterHandler(chros.ChROSClockHandler(publish_rate))

    # b) body state for `box`
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(publish_rate, box, "~/box")
    )

    # c) TF between floor and box
    tfh = chros.ChROSTFHandler(publish_rate)
    tfh.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tfh)

    # d) our custom integer publisher
    custom_h = MyCustomHandler("~/my_topic", publish_rate)
    ros_manager.RegisterHandler(custom_h)

    ros_manager.Initialize()

    # 8) Simulation + rendering control variables
    time       = 0.0
    time_step  = 1e-3   # simulation time step
    time_end   = 30.0   # total sim time
    step_number       = 0
    render_frame_rate = 30.0                           # desired fps
    render_steps      = int(1.0 / time_step / render_frame_rate)

    realtime_timer = ch.ChRealtimeStepTimer()

    # 9) Main loop: advance sim, ROS, and do conditional rendering
    while app.GetDevice().run() and time < time_end:
        # advance physics
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        # publish to ROS
        if not ros_manager.Update(time, time_step):
            print("[main] ROS manager signalled an error, shutting down.")
            break

        # every render_steps steps we redraw
        if step_number % render_steps == 0:
            app.BeginScene(True, True, irr.SColor(255, 100, 100, 140))
            app.DrawAll()
            app.EndScene()

        realtime_timer.Spin(time_step)
        step_number += 1

    # 10) clean up ROS
    rclpy.shutdown()


if __name__ == "__main__":
    main()