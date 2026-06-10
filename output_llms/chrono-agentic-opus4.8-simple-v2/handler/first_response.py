import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher                                              # rclpy publisher type hint
from std_msgs.msg import Int64                                      # integer ROS message


class MyCustomHandler(chros.ChROSHandler):                          # custom integer-publishing handler

    def __init__(self, topic):
        super().__init__(25)                                       # publish at 25 Hz
        self.topic = topic                                         # ROS topic to publish on
        self.publisher: rclpy.publisher.Publisher = None           # created in Initialize
        self.ticker = 0                                            # integer payload counter

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)  # make publisher on the node
        return True                                                # report success or the handler is dropped

    def Tick(self, time: float):
        msg = Int64()                                              # build the integer message
        msg.data = self.ticker                                     # set the payload
        self.publisher.publish(msg)                               # publish to the topic
        self.ticker += 1                                          # advance the counter


def main():
    sys = chrono.ChSystemNSC()                                     # rigid-body system
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g = 9.81 down (Z-up)
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # contact present -> Bullet collision

    contact_mat = chrono.ChContactMaterialNSC()                    # shared contact material
    contact_mat.SetFriction(0.5)                                   # Coulomb friction
    contact_mat.SetRestitution(0.0)                                # no bounce

    floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, contact_mat)  # fixed floor slab
    floor.SetPos(chrono.ChVector3d(0, 0, -0.5))                    # top face at z = 0
    floor.SetFixed(True)                                           # floor is immovable
    floor.SetName("base_link")                                     # TF root frame name
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # floor texture
    sys.Add(floor)                                                 # add floor to system

    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, contact_mat)  # movable unit box
    box.SetPos(chrono.ChVector3d(0, 0, 5))                         # drop from 5 m up
    box.SetName("box")                                            # TF child frame name
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # box texture
    sys.Add(box)                                                   # add box to system

    ros_manager = chros.ChROSPythonManager()                       # Python ROS manager (hosts Python handlers)
    ros_manager.RegisterHandler(chros.ChROSClockHandler())         # /clock first -> graph time-synced to sim
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box/state"))  # publish box pose/twist at 25 Hz

    tf_handler = chros.ChROSTFHandler(30)                          # transform tree at 30 Hz
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())  # base_link -> box transform
    ros_manager.RegisterHandler(tf_handler)                        # register the TF handler

    custom_handler = MyCustomHandler("~/my_topic")                 # custom integer publisher
    ros_manager.RegisterPythonHandler(custom_handler)              # Python subclass -> RegisterPythonHandler

    ros_manager.Initialize()                                       # initialize ONCE, after all registration

    vis = chronoirr.ChVisualSystemIrrlicht()                       # Irrlicht render window
    vis.AttachSystem(sys)                                          # bind the system's visual assets
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)              # Z-up camera
    vis.SetWindowSize(1280, 720)                                   # window resolution
    vis.SetWindowTitle("ROS Handler Demo")                         # window title
    vis.Initialize()                                               # create the device FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
    vis.AddSkyBox()                                                # sky box
    vis.AddCamera(chrono.ChVector3d(8, -8, 5), chrono.ChVector3d(0, 0, 1))  # eye, look-at
    vis.AddTypicalLights()                                         # standard lighting

    time_step = 1e-3                                               # integration step [s]
    sim_end = 10.0                                                 # stop time [s]
    render_fps = 50.0                                              # frames per simulated second
    render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps per rendered frame
    realtime_timer = chrono.ChRealtimeStepTimer()                 # keep wall-clock ~ sim time
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()                                          # begin frame
        vis.Render()                                             # draw the scene
        vis.EndScene()                                           # end frame
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)                        # advance one physics step
            time = sys.GetChTime()                               # current sim time
            if not ros_manager.Update(time, time_step):          # pump ROS handlers; break if graph is down
                break
            realtime_timer.Spin(time_step)                       # throttle to real time
            if sys.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
