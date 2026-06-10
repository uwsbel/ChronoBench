import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    # custom ROS handler publishing an incrementing integer message

    def __init__(self, topic):
        super().__init__(1)                                           # publish rate: 1 Hz
        self.topic = topic                                           # ROS topic name
        self.publisher = None                                        # rclpy publisher (set in Initialize)
        self.ticker = 0                                              # running integer counter

    def Initialize(self, interface):
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)  # create the publisher
        print("Created custom handler publishing to", self.topic)    # log the registration
        return True                                                 # must return True on success

    def Tick(self, time):
        msg = Int64()                                               # integer message
        msg.data = self.ticker                                      # current counter value
        self.publisher.publish(msg)                                # publish to the topic
        self.ticker += 1                                           # advance the counter


def main():
    sys = chrono.ChSystemNSC()                                      # non-smooth contact system
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g = 9.81 along -Z
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Bullet collision (contact scene)

    mat = chrono.ChContactMaterialNSC()                            # NSC contact material
    mat.SetFriction(0.5)                                           # friction coefficient
    mat.SetRestitution(0.0)                                        # no bounce

    floor = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, mat)  # fixed floor box
    floor.SetPos(chrono.ChVector3d(0, 0, -1))                      # top surface at z = -0.5
    floor.SetFixed(True)                                           # floor is immovable
    floor.SetName("base_link")                                    # TF root frame name
    sys.Add(floor)                                                # add floor to the system

    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, mat)     # movable box body
    box.SetPos(chrono.ChVector3d(0, 0, 1.5))                       # spawned just above the floor (lands and rests)
    box.SetName("box")                                           # TF child frame name
    sys.Add(box)                                                  # add box to the system

    ros_manager = chros.ChROSPythonManager()                       # Python ROS manager (hosts Python handlers)
    ros_manager.RegisterHandler(chros.ChROSClockHandler())         # /clock handler FIRST
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box_state"))  # publish box pose/twist
    tf_handler = chros.ChROSTFHandler(30)                          # transform handler at 30 Hz
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())  # floor -> box transform
    ros_manager.RegisterHandler(tf_handler)                        # register the TF handler
    ros_manager.RegisterPythonHandler(MyCustomHandler("~/my_topic"))  # register the custom Python handler
    ros_manager.Initialize()                                       # initialize ONCE, after all registration

    vis = chronoirr.ChVisualSystemIrrlicht()                       # Irrlicht visualization
    vis.AttachSystem(sys)                                          # bind the system
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)              # Z-up camera
    vis.SetWindowSize(1280, 720)                                  # window size
    vis.SetWindowTitle("ROS Handler Demo")                        # window title
    vis.Initialize()                                              # create device FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
    vis.AddSkyBox()                                               # sky box
    vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 1))  # camera eye/target
    vis.AddTypicalLights()                                        # standard lights

    time_step = 1e-3                                              # integration step [s]
    sim_end = 10.0                                                # stop time [s]
    render_fps = 50.0                                             # review frame rate
    render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps per rendered frame
    realtime_timer = chrono.ChRealtimeStepTimer()                # keeps wall-clock ~ sim time
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()                                         # begin frame
        vis.Render()                                             # render scene
        vis.EndScene()                                           # end frame
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)                       # advance one physics step
            time = sys.GetChTime()                              # current sim time
            if not ros_manager.Update(time, time_step):        # pump ROS handlers; break on shutdown
                break
            realtime_timer.Spin(time_step)                     # throttle to real time
            if sys.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
