import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64

publish_rate = 10                                                   # ROS handler publish rate [Hz]


class MyCustomHandler(chros.ChROSHandler):                          # custom ROS publisher handler

    def __init__(self, topic):
        super().__init__(publish_rate)                             # publish rate in Hz
        self.topic = topic                                         # ROS topic name
        self.publisher: rclpy.publisher.Publisher = None           # set in Initialize
        self.ticker = 0                                            # message counter

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)   # rclpy publisher
        print("Created publisher for topic: " + self.topic)        # report registration
        return True                                               # success

    def Tick(self, time: float):
        msg = Int64()                                             # std_msgs/Int64
        msg.data = self.ticker                                    # payload = tick count
        self.publisher.publish(msg)                              # publish to the topic
        self.ticker += 1                                         # advance the counter


def main():
    sys = chrono.ChSystemNSC()                                   # rigid-body NSC system
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g = 9.81 down (Z-up)
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required: floor/box contact

    mat = chrono.ChContactMaterialNSC()                         # NSC contact material
    mat.SetFriction(0.5)                                        # friction coefficient

    floor = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, mat)   # floor: 20x20x1 box
    floor.SetPos(chrono.ChVector3d(0, 0, -1))                  # top surface at z = -0.5
    floor.SetFixed(True)                                       # floor is static
    floor.SetName("base_link")                                # TF root frame
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # floor texture
    sys.Add(floor)                                            # add floor to system

    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, mat)  # falling box: 1x1x1
    box.SetPos(chrono.ChVector3d(0, 0, 5))                    # start above the floor
    box.SetName("box")                                        # TF child frame
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # box texture
    sys.Add(box)                                              # add box to system

    vis = chronoirr.ChVisualSystemIrrlicht()                  # Irrlicht render window
    vis.AttachSystem(sys)                                     # bind to the system
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)        # Z-up camera
    vis.SetWindowSize(1280, 720)                             # window resolution
    vis.SetWindowTitle("ROS Handler Demo")                  # window title
    vis.Initialize()                                        # create device FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
    vis.AddSkyBox()                                         # sky box
    vis.AddCamera(chrono.ChVector3d(-8, -8, 4), chrono.ChVector3d(0, 0, 0))  # camera eye/target
    vis.AddTypicalLights()                                  # standard lights

    ros_manager = chros.ChROSPythonManager()               # Python ROS manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  # /clock first

    body_handler = chros.ChROSBodyHandler(publish_rate, box, "~/box")  # box pose/twist
    ros_manager.RegisterHandler(body_handler)              # register body handler

    tf_handler = chros.ChROSTFHandler(publish_rate)        # /tf publisher
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())  # base_link -> box
    ros_manager.RegisterHandler(tf_handler)               # register TF handler

    custom_handler = MyCustomHandler("~/my_topic")        # custom Int64 publisher
    ros_manager.RegisterPythonHandler(custom_handler)     # Python handler registration

    ros_manager.Initialize()                              # initialize ONCE, after all handlers

    time_step = 1e-3                                       # integration step [s]
    sim_end = 10.0                                         # simulation duration [s]
    render_fps = 50.0                                      # review frame rate
    render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged render cadence

    while vis.Run() and sys.GetChTime() < sim_end:        # render-cadence main loop
        vis.BeginScene()                                  # begin frame
        vis.Render()                                      # draw scene
        vis.EndScene()                                    # end frame
        for _ in range(render_every):                     # advance physics batch
            sys.DoStepDynamics(time_step)                 # step the dynamics
            time = sys.GetChTime()                        # current sim time
            if not ros_manager.Update(time, time_step):   # publish ROS state (scored core)
                break                                     # stop if ROS shut down
            if sys.GetChTime() >= sim_end:                # reached end of sim
                break


if __name__ == "__main__":
    main()
