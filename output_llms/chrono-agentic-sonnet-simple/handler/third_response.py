import os
import rclpy
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64  # custom handler publishes an Int64 counter


class MyCustomHandler(chros.ChROSHandler):
    """Custom ROS handler that publishes a tick counter over ROS2."""

    def __init__(self, topic):
        super().__init__(10)  # publish at 10 Hz (publish_rate)
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True  # must return True on success

    def Tick(self, time: float):
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1


def main():
    # --- system ---
    sys = chrono.ChSystemNSC()  # NSC rigid-body system
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up gravity
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for contact

    # --- contact material ---
    mat = chrono.ChContactMaterialNSC()  # NSC contact material

    # --- floor (fixed) ---
    floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, mat)  # 10×1×10 floor
    floor.SetPos(chrono.ChVector3d(0, -0.5, 0))  # center at y=-0.5 so top face is at y=0
    floor.SetFixed(True)
    floor.SetName("base_link")  # TF root frame name
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    sys.Add(floor)

    # --- box (dynamic) ---
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, mat)  # 1×1×1 box
    box.SetPos(chrono.ChVector3d(0, 5, 0))  # drop from y=5
    box.SetName("box")  # TF child frame name
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/redwhite.png"))
    sys.Add(box)

    # --- Irrlicht visualization (Initialize first, add scene elements after) ---
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up scene
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("handler demo")
    vis.Initialize()  # FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(6, 6, 0), chrono.ChVector3d(0, 0, 0))  # side camera
    vis.AddTypicalLights()

    # --- ROS manager ---
    ros_manager = chros.ChROSPythonManager()

    # clock handler first
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  # publishes /clock

    # body handler — publishes box pose at 10 Hz
    publish_rate = 10  # Hz
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/output/box/state"))

    # TF handler — publishes floor→box transform
    tf_handler = chros.ChROSTFHandler(publish_rate)  # 10 Hz TF
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())  # base_link → box
    ros_manager.RegisterHandler(tf_handler)

    # custom Python handler — publishes Int64 counter at 10 Hz
    ros_manager.RegisterPythonHandler(MyCustomHandler("~/output/custom/data"))

    # initialize ROS after all handlers are registered
    ros_manager.Initialize()

    # --- simulation parameters ---
    time_step = 1e-3  # 1 ms physics step
    sim_end = 30.0    # 30 s simulation
    render_fps = 50.0
    render_every = max(1, round(1.0 / (render_fps * time_step)))  # cadence constant
    step_number = 0                                                # step counter for conditional rendering
    render_step_size = render_every                                # steps between renders (= render_every)
    render_steps = render_step_size                                # alias used in loop condition

    while vis.Run() and sys.GetChTime() < sim_end:
        if step_number % render_steps == 0:  # conditional rendering — update every render_steps
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()
        step_number += 1
        if not ros_manager.Update(time, time_step):  # pump ROS handlers after physics step
            break


if __name__ == "__main__":
    main()
