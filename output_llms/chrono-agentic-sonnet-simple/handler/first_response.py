import os
import rclpy
import rclpy.publisher
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
from std_msgs.msg import Int64                           # custom handler publishes integer messages


class MyCustomHandler(chros.ChROSHandler):
    """Publish incrementing Int64 messages to a custom topic."""

    def __init__(self, topic):
        super().__init__(1)                              # publish at 1 Hz
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True                                      # must return True on success

    def Tick(self, time: float):
        msg = Int64()
        msg.data = self.ticker                           # increment per publish
        self.publisher.publish(msg)
        self.ticker += 1


def main():
    time_step = 1e-3                                     # physics step size [s]
    sim_end = 10.0                                       # simulation end time [s]
    render_fps = 50.0                                    # target render frame rate

    # --- system ---
    sys = chrono.ChSystemNSC()                           # NSC for rigid body contact
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # g = 9.81 m/s^2 downward
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # collision detection

    # --- contact material ---
    mat = chrono.ChContactMaterialNSC()                  # NSC material
    mat.SetFriction(0.5)                                 # friction coefficient
    mat.SetRestitution(0.1)                              # low restitution

    # --- floor (fixed) ---
    floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, mat)  # wide flat floor
    floor.SetPos(chrono.ChVector3d(0, -0.5, 0))         # center at y=-0.5 so top face at y=0
    floor.SetFixed(True)                                  # immovable ground
    floor.SetName("base_link")                           # TF root frame name
    sys.Add(floor)

    # --- movable box ---
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, mat)  # 1m cube
    box.SetPos(chrono.ChVector3d(0, 5, 0))              # start above the floor
    box.SetFixed(False)                                  # dynamic body
    box.SetName("box")                                   # TF child frame name
    sys.Add(box)

    # --- Irrlicht visualization ---
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)    # Y-up for this MBS scene
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ROS Handler Demo")
    vis.Initialize()                                     # Initialize FIRST, then add scene elements
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(8, 6, -8), chrono.ChVector3d(0, 2, 0))
    vis.AddTypicalLights()

    # --- ROS manager ---
    ros_manager = chros.ChROSPythonManager()             # Python manager required for custom handlers

    # clock handler first (time-syncs the ROS graph)
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    # body handler: publishes pose/twist of the movable box
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/output/box/state"))

    # TF handler: publishes TF frames for floor and box
    tf_handler = chros.ChROSTFHandler(30)                # 30 Hz TF publish rate
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    # custom handler: publishes incrementing Int64 messages
    ros_manager.RegisterPythonHandler(MyCustomHandler("~/output/custom/integer"))

    ros_manager.Initialize()                             # exactly once, after all registration

    # --- render cadence ---
    render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps per render frame

    # --- review-only recording setup ---

    # --- main simulation loop ---
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)                # advance physics
            time = sys.GetChTime()
            if not ros_manager.Update(time, time_step): # pump ROS handlers; break on shutdown
                break
            if sys.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
