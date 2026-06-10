import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import String                          # String message for the custom handler


class MyCustomHandler(chros.ChROSHandler):                # custom Python ROS handler publishing a string

    def __init__(self, topic):
        super().__init__(1)                               # publish rate: 1 Hz
        self.topic = topic                                # ROS topic name
        self.publisher: rclpy.publisher.Publisher = None  # created in Initialize
        self.ticker = 0                                   # tick counter appended to the message
        self.message = "Hello, world! At time: "          # fixed message prefix

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)   # std_msgs/String publisher
        return True                                       # must return True on success

    def Tick(self, time: float):
        msg = String()                                    # build a String message
        msg.data = self.message + str(self.ticker)        # concatenate prefix and tick count
        self.publisher.publish(msg)                       # publish to the topic
        self.ticker += 1                                  # advance the ticker


def main():
    sys = chrono.ChSystemNSC()                            # NSC system for rigid contact
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # gravity, Z-up
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # collision needed: floor + box contact

    floor_mat = chrono.ChContactMaterialNSC()            # contact material for the floor
    floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, floor_mat)   # floor box (full extents)
    floor.SetPos(chrono.ChVector3d(0, 0, -1))            # sink the floor so its top is at z=-0.5
    floor.SetFixed(True)                                  # floor is static
    floor.SetName("base_link")                            # TF root frame name
    sys.Add(floor)                                        # add floor to the system

    box_mat = chrono.ChContactMaterialNSC()              # contact material for the box
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, box_mat)   # falling box (full extents)
    box.SetPos(chrono.ChVector3d(0, 0, 5))              # start the box above the floor
    box.SetName("box")                                   # TF child frame name
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.2, 0.2))   # tint the box
    sys.Add(box)                                          # add box to the system

    ros_manager = chros.ChROSPythonManager()             # Python manager hosts the custom handler
    ros_manager.RegisterHandler(chros.ChROSClockHandler())   # /clock first, time-syncs the ROS graph

    body_handler = chros.ChROSBodyHandler(25, box, "~/box")   # publish the box pose/twist at 25 Hz
    ros_manager.RegisterHandler(body_handler)            # register the body handler

    tf_handler = chros.ChROSTFHandler(30)                # TF tree at 30 Hz
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())   # base_link -> box transform
    ros_manager.RegisterHandler(tf_handler)              # register the TF handler

    custom_handler = MyCustomHandler("~/my_topic")       # custom string-publishing handler
    ros_manager.RegisterPythonHandler(custom_handler)    # Python subclass -> RegisterPythonHandler

    ros_manager.Initialize()                             # initialize once, after all handlers

    vis = chronoirr.ChVisualSystemIrrlicht()             # Irrlicht render window
    vis.AttachSystem(sys)                                # bind the system to the visualizer
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)    # Z-up camera
    vis.SetWindowSize(1280, 720)                         # window resolution
    vis.SetWindowTitle("ROS Custom Handler")             # window title
    vis.Initialize()                                     # create the device first
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo overlay
    vis.AddSkyBox()                                       # sky box
    vis.AddCamera(chrono.ChVector3d(0, -8, 4), chrono.ChVector3d(0, 0, 0))   # camera eye/target
    vis.AddTypicalLights()                               # standard lighting

    time_step = 1e-3                                      # integration step
    sim_end = 30.0                                        # simulation duration
    render_fps = 50.0                                     # review frame rate
    render_every = max(1, round(1.0 / (render_fps * time_step)))   # physics steps per rendered frame
    realtime_timer = chrono.ChRealtimeStepTimer()        # keep wall-clock ~ sim time

    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()                                 # begin frame
        vis.Render()                                     # draw the scene
        vis.EndScene()                                   # end frame
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)                # advance physics one step
            time = sys.GetChTime()                       # current sim time
            if not ros_manager.Update(time, time_step):  # publish ROS state; break if ROS shut down
                break
            realtime_timer.Spin(time_step)               # throttle to real time
            if sys.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
