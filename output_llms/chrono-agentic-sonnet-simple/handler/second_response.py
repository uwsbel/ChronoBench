import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import String                               # String message type


class MyCustomHandler(chros.ChROSHandler):

    def __init__(self, topic):
        super().__init__(1)                                   # publish at 1 Hz
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0
        self.message = "Hello, world! At time: "             # message attribute

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)  # String publisher
        return True

    def Tick(self, time: float):
        msg = String()
        msg.data = self.message + str(self.ticker)            # concatenated string message
        self.publisher.publish(msg)
        self.ticker += 1


def main():
    sys = chrono.ChSystemNSC()                                # NSC system for rigid-body scene
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # g = 9.81 Y-down
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for collision

    mat = chrono.ChContactMaterialNSC()                       # NSC contact material

    floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, mat)  # floor body
    floor.SetPos(chrono.ChVector3d(0, -0.5, 0))              # place at Y=-0.5
    floor.SetFixed(True)                                      # fix floor
    floor.SetName("base_link")                                # TF root name
    sys.AddBody(floor)

    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, mat)  # box body
    box.SetPos(chrono.ChVector3d(0, 1.5, 0))                 # spawn above floor
    box.SetName("box")                                        # TF child name
    sys.AddBody(box)

    ros_manager = chros.ChROSPythonManager()                  # Python manager (required for custom handler)

    ros_manager.RegisterHandler(chros.ChROSClockHandler())    # clock handler first

    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, box, "~/output/body/state")  # body pose/twist publisher
    )

    tf_handler = chros.ChROSTFHandler(30)                     # TF handler at 30 Hz
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())  # floor -> box transform
    ros_manager.RegisterHandler(tf_handler)

    ros_manager.RegisterPythonHandler(MyCustomHandler("~/my_topic"))  # custom String handler

    ros_manager.Initialize()                                  # initialize ONCE after all registration

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)         # Y-up convention
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ROS Handler Demo - String Publisher")
    vis.Initialize()                                          # Initialize FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(6, 4, 6), chrono.ChVector3d(0, 0, 0))  # eye, target
    vis.AddTypicalLights()

    time_step = 1e-3                                          # physics time step [s]
    sim_end = 10.0                                            # simulation end time [s]
    render_fps = 50.0
    render_every = max(1, round(1.0 / (render_fps * time_step)))  # cadence constant


    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)                    # advance physics
            time = sys.GetChTime()
            if not ros_manager.Update(time, time_step):      # update ROS handlers; break on shutdown
                break
            if sys.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
