import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import String                                       # String message type


class MyCustomHandler(chros.ChROSHandler):                            # custom ROS handler publishing a String

    def __init__(self, topic):
        super().__init__(1)                                          # publish at 1 Hz
        self.topic = topic                                          # ROS topic name
        self.publisher: rclpy.publisher.Publisher = None            # created in Initialize
        self.ticker = 0                                            # incrementing counter
        self.message = "Hello, world! At time: "                  # message prefix

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)   # String publisher
        return True                                                 # must return True on success

    def Tick(self, time: float):
        msg = String()                                             # std_msgs/String
        msg.data = self.message + str(self.ticker)                 # concatenated string payload
        self.publisher.publish(msg)                                # publish the message
        print(msg.data)                                            # log the published string
        self.ticker += 1                                          # advance the counter


def main():
    sys = chrono.ChSystemNSC()                                     # NSC rigid-body system
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # g = 9.81 along -Z
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # collision needed for floor/box contact

    floor_mat = chrono.ChContactMaterialNSC()                      # contact material for the floor

    floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, floor_mat)   # floor plate
    floor.SetPos(chrono.ChVector3d(0, 0, -1))                      # placed below the box
    floor.SetFixed(True)                                          # floor is static
    floor.SetName("base_link")                                    # TF root frame
    sys.Add(floor)                                                # add floor to system

    box_mat = chrono.ChContactMaterialNSC()                       # contact material for the box

    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, box_mat)   # falling box
    box.SetPos(chrono.ChVector3d(0, 0, 5))                        # start above the floor
    box.SetName("box")                                            # TF child frame
    sys.Add(box)                                                  # add box to system

    vis = chronoirr.ChVisualSystemIrrlicht()                      # Irrlicht render window
    vis.AttachSystem(sys)                                         # bind the system
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)             # Z-up camera
    vis.SetWindowSize(1280, 720)                                  # window resolution
    vis.SetWindowTitle("ROS Custom Handler")                      # window title
    vis.Initialize()                                             # create device FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
    vis.AddSkyBox()                                              # sky box
    vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))   # camera eye/target
    vis.AddTypicalLights()                                       # standard lights

    ros_manager = chros.ChROSPythonManager()                     # Python ROS manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())       # /clock first

    body_handler = chros.ChROSBodyHandler(25, box, "~/box")      # publish box pose/twist
    ros_manager.RegisterHandler(body_handler)                    # register body handler

    tf_handler = chros.ChROSTFHandler(30)                        # TF tree handler
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())   # base_link -> box
    ros_manager.RegisterHandler(tf_handler)                      # register TF handler

    custom_handler = MyCustomHandler("~/my_topic")               # custom String handler
    ros_manager.RegisterPythonHandler(custom_handler)            # register Python subclass

    ros_manager.Initialize()                                     # initialize ONCE after all handlers

    time_step = 1e-3                                             # integration step
    sim_end = 10.0                                               # simulation duration
    render_fps = 50.0                                            # frame rate for review
    render_every = max(1, round(1.0 / (render_fps * time_step)))   # steps per rendered frame

    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()                                        # begin frame
        vis.Render()                                            # render scene
        vis.EndScene()                                          # end frame
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)                       # advance physics one step
            time = sys.GetChTime()                              # current sim time
            if not ros_manager.Update(time, time_step):        # pump ROS handlers; break on shutdown
                break
            if sys.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
