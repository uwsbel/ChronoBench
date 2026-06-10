import os
import pychrono.core as chrono
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import String                                      # custom handler message type


class MyCustomHandler(chros.ChROSHandler):                          # custom Python ChROSHandler subclass

    def __init__(self, topic):
        super().__init__(1)                                         # publish rate = 1 Hz
        self.topic = topic                                          # ROS topic to publish on
        self.publisher: rclpy.publisher.Publisher = None           # created in Initialize
        self.ticker = 0                                            # self-incrementing counter
        self.message = "Hello, world! At time: "                   # message prefix

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)   # publisher on the rclpy node
        return True                                                # must return True or the handler is dropped

    def Tick(self, time: float):
        msg = String()                                            # string message
        msg.data = self.message + str(self.ticker)                # concatenated payload
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
    sys.Add(floor)

    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)   # movable box
    box.SetPos(chrono.ChVector3d(0, 0, 5))                        # dropped from above
    box.SetRot(chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(1, 0, 0)))   # slight tilt
    box.SetName("box")                                           # TF child frame
    sys.Add(box)

    ros_manager = chros.ChROSPythonManager()                     # Python manager hosts the custom handler
    ros_manager.RegisterHandler(chros.ChROSClockHandler())       # clock FIRST -> /clock
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))   # box pose/twist at 25 Hz
    tf_handler = chros.ChROSTFHandler(30)                        # TF tree at 30 Hz
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())   # base_link -> box
    ros_manager.RegisterHandler(tf_handler)
    ros_manager.RegisterPythonHandler(MyCustomHandler("~/my_topic"))   # custom String publisher
    ros_manager.Initialize()                                     # ONCE, after all registration

    time_step = 1e-3                                             # integration step
    time_end = 30                                                # simulation duration [s]
    realtime_timer = chrono.ChRealtimeStepTimer()               # keeps wall-clock ~ sim time

    time = 0
    while time < time_end:                                       # plain-MBS headless loop
        sys.DoStepDynamics(time_step)                            # advance physics
        time = sys.GetChTime()                                   # current sim time
        if not ros_manager.Update(time, time_step):             # publish all handlers; break on shutdown
            break
        realtime_timer.Spin(time_step)                          # real-time pacing


if __name__ == "__main__":
    main()
