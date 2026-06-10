import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64                                        # custom handler message type


class MyCustomHandler(chros.ChROSHandler):                          # custom Python ChROSHandler subclass

    def __init__(self, topic):
        super().__init__(1)                                         # publish rate = 1 Hz
        self.topic = topic                                          # ROS topic to publish on
        self.publisher: rclpy.publisher.Publisher = None           # created in Initialize
        self.ticker = 0                                            # self-incrementing counter

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)   # publisher on the rclpy node
        return True                                                # must return True or the handler is dropped

    def Tick(self, time: float):
        msg = Int64()                                              # integer message
        msg.data = self.ticker                                     # current counter value
        self.publisher.publish(msg)                                # publish to the topic
        self.ticker += 1                                          # advance the counter


def main():
    sys = chrono.ChSystemNSC()                                    # NSC system for rigid contact
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # gravity, Z-up
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED (floor+box contact)

    phys_mat = chrono.ChContactMaterialNSC()                      # shared contact material
    phys_mat.SetFriction(0.5)                                     # friction coefficient

    floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)   # fixed floor
    floor.SetPos(chrono.ChVector3d(0, 0, -1))                     # below the box
    floor.SetFixed(True)                                          # immovable ground
    floor.SetName("base_link")                                    # TF root frame
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))   # floor texture
    sys.Add(floor)

    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)   # movable box
    box.SetPos(chrono.ChVector3d(0, 0, 5))                        # dropped from above
    box.SetRot(chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(1, 0, 0)))   # slight tilt
    box.SetName("box")                                           # TF child frame
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))   # box texture
    sys.Add(box)

    publish_rate = 10                                            # ROS handler publish rate [Hz]

    ros_manager = chros.ChROSPythonManager()                     # Python manager hosts the custom handler
    ros_manager.RegisterHandler(chros.ChROSClockHandler())       # clock FIRST -> /clock
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))   # box pose/twist
    tf_handler = chros.ChROSTFHandler(publish_rate)             # TF tree
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())   # base_link -> box
    ros_manager.RegisterHandler(tf_handler)
    ros_manager.RegisterPythonHandler(MyCustomHandler("~/my_topic"))   # custom Int64 publisher
    ros_manager.Initialize()                                     # ONCE, after all registration

    vis = chronoirr.ChVisualSystemIrrlicht()                     # Irrlicht window
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)           # Z-up, BEFORE Initialize
    vis.SetWindowSize(1280, 720)                                # window dimensions
    vis.SetWindowTitle("ROS Custom Handler")
    vis.Initialize()                                            # device created FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # after Initialize
    vis.AddSkyBox()                                            # sky box
    vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))   # eye, target
    vis.AddTypicalLights()                                     # standard lights

    time_step = 1e-3                                            # integration step
    render_step_size = 1.0 / 25                                 # render every 1/25 s
    render_steps = max(1, round(render_step_size / time_step))   # physics steps per render
    step_number = 0                                            # physics step counter
    realtime_timer = chrono.ChRealtimeStepTimer()             # keeps wall-clock ~ sim time

    while vis.Run():                                          # real-time loop
        if step_number % render_steps == 0:                   # conditional rendering every few steps
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        sys.DoStepDynamics(time_step)                         # advance physics
        time = sys.GetChTime()                                # current sim time
        if not ros_manager.Update(time, time_step):           # publish all handlers; break on shutdown
            break
        step_number += 1                                     # advance step counter
        realtime_timer.Spin(time_step)                       # real-time pacing


if __name__ == "__main__":
    main()
