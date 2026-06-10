import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64

publish_rate = 10                                                    # 10 Hz ROS publish rate for all handlers


class MyCustomHandler(chros.ChROSHandler):                            # custom ROS publisher handler

    def __init__(self, topic):
        super().__init__(publish_rate)                               # publish at publish_rate Hz
        self.topic = topic                                          # topic name to publish on
        self.publisher = None                                       # created in Initialize
        self.ticker = 0                                             # message counter

    def Initialize(self, interface):                                # called once when the manager starts
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)  # rclpy publisher
        print("Created publisher for topic: " + self.topic)         # diagnostic
        return True                                                 # MUST return True or the handler is dropped

    def Tick(self, time):                                           # called at publish_rate Hz
        msg = Int64()                                              # std_msgs/Int64 message
        msg.data = self.ticker                                     # payload = tick count
        self.publisher.publish(msg)                                # publish it
        self.ticker += 1                                           # advance counter


def main():
    sys = chrono.ChSystemNSC()                                     # rigid-body NSC system
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g = 9.81 along -Z (Z-up)
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required: scene has contact

    floor_mat = chrono.ChContactMaterialNSC()                      # contact material for the floor

    floor = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, floor_mat)  # 20x20x1 floor slab
    floor.SetPos(chrono.ChVector3d(0, 0, -1))                      # top face at z = -0.5
    floor.SetFixed(True)                                           # floor is immovable
    floor.SetName("base_link")                                     # conventional TF root frame
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # floor texture
    sys.Add(floor)                                                 # add floor to the system

    box_mat = chrono.ChContactMaterialNSC()                        # contact material for the box

    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, box_mat)  # 1 m cube, density 1000
    box.SetPos(chrono.ChVector3d(0, 0, 5))                         # drop from z = 5
    box.SetName("box")                                            # TF child frame name
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # box texture
    sys.Add(box)                                                   # add box to the system

    ros_manager = chros.ChROSPythonManager()                       # Python ROS manager (hosts python handlers)
    ros_manager.RegisterHandler(chros.ChROSClockHandler())         # /clock first — time-sync the graph

    body_handler = chros.ChROSBodyHandler(publish_rate, box, "~/box")  # publish box pose/twist
    ros_manager.RegisterHandler(body_handler)                      # register the body handler

    tf_handler = chros.ChROSTFHandler(publish_rate)          # publishes /tf transforms
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())  # base_link -> box
    ros_manager.RegisterHandler(tf_handler)                        # register the TF handler

    custom_handler = MyCustomHandler("~/my_topic")                 # custom Int64 publisher
    ros_manager.RegisterPythonHandler(custom_handler)              # python subclass -> RegisterPythonHandler

    ros_manager.Initialize()                                       # initialize ONCE after all registration

    vis = chronoirr.ChVisualSystemIrrlicht()                       # Irrlicht render window
    vis.AttachSystem(sys)                                          # bind the system
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)             # Z-up camera
    vis.SetWindowSize(1280, 720)                                   # window resolution
    vis.SetWindowTitle("ROS Handler Demo")                        # window title
    vis.Initialize()                                              # create device FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
    vis.AddSkyBox()                                               # sky box
    vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))  # eye, target
    vis.AddTypicalLights()                                        # standard lights

    time_step = 1e-3                                               # integration step
    sim_end = 10.0                                                 # stop time
    render_fps = 25.0                                             # review render frame rate
    render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant
    while vis.Run() and sys.GetChTime() < sim_end:               # outer render loop
        vis.BeginScene()                                         # begin frame
        vis.Render()                                            # draw the scene
        vis.EndScene()                                          # end frame
        for _ in range(render_every):                          # inner physics batch
            sys.DoStepDynamics(time_step)                      # advance one physics step
            time = sys.GetChTime()                             # current sim time
            if not ros_manager.Update(time, time_step):        # pump ROS handlers; break if shut down
                break
            if sys.GetChTime() >= sim_end:                     # stop at sim_end
                break


if __name__ == "__main__":
    main()
